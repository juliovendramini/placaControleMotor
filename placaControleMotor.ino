// ================================================================
// Controle de Motor DC com Encoder - ATtiny1616
// Comunicação half-duplex one-wire (barramento compartilhado)
// PID baseado na placaBrickVendra com biblioteca PID_v1
// ================================================================

#include <EEPROM.h>
#include <avr/wdt.h>
#include "PID_v1.h"

// --- Pinos do motor (H-bridge) ---
#define PINO1_MOTOR 1   // PA5 (PWM frente)
#define PINO2_MOTOR 0   // PA4 (PWM trás)
// Encoder: PB4 (canal A) e PB5 (canal B)
#define LED 2

// Frequências PWM aprox. a 20MHz:
//   TCA_SPLIT_CLKSEL_DIV64_gc   → ~1225 Hz
//   TCA_SPLIT_CLKSEL_DIV256_gc  →  ~306 Hz
//   TCA_SPLIT_CLKSEL_DIV1024_gc →   ~76 Hz
#define PWM_PRESCALER TCA_SPLIT_CLKSEL_DIV1024_gc

// --- Protocolo (6 bytes comando, 8 bytes resposta) ---
// Comando:
//   [0] ID do equipamento
//   [1] Modo/Tipo de mensagem
//   [2] Velocidade (int8_t, -100 a 100) ou dado HIGH
//   [3] Giro comando HIGH byte (ou dado)
//   [4] Giro comando LOW byte (ou dado)
//   [5] CRC = XOR bytes 0..4
//
// Resposta (8 bytes):
//   [0] ID
//   [1] Modo atual
//   [2] Estado do motor (0=parado, 1=horário, 2=anti-horário)
//   [3..6] Contagem absoluta de pulsos (int32_t, big-endian)
//   [7] CRC = XOR bytes 0..6

#define TAMANHO_PACOTE_CMD  6
#define TAMANHO_PACOTE_RESP 8
#define TIMEOUT_PACOTE 1000       // us - reseta recepção se byte demorar

// Modos de operação
#define MODO_PID           0    // Controle PID de velocidade (-100..100)
#define MODO_PWM           1    // PWM direto/potência (-100..100)
#define MODO_SET_FREIO     2    // Configura modo de frenagem
#define MODO_CALIBRACAO    4    // Calibração de velocidade máxima
#define MODO_RESET         3    // Reset geral (volta ao estado inicial)
#define MODO_SET_KP        5    // Configura Kp (buf[2:3] = valor × 100)
#define MODO_SET_KI        6    // Configura Ki (buf[2:3] = valor × 100)
#define MODO_SET_KD        7    // Configura Kd (buf[2:3] = valor × 100)
#define MODO_SET_CALIBRACAO 8   // Forçar calibração: buf[2]=dir(0=hor,1=anti), buf[3:4]=valor
#define MODO_ATUALIZA      9    // Apenas lê estado atual
#define MODO_GET_CALIBRACAO 10  // Lê calibração atual sem alterar
#define MODO_SET_KP_FREIO  11   // Configura Kp do freio travado (buf[2:3] = valor × 100)
#define MODO_SET_KD_FREIO  12   // Configura Kd do freio travado (buf[2:3] = valor × 100)
#define MODO_SET_DELTA_FREIO 13  // Configura DELTA do freio travado (buf[2:3] = valor direto)

// --- Calibração ---
#define TEMPO_ESPERA_PARAR    2000  // ms esperando o motor parar
#define QUANTIDADE_REPETICAO_CALIBRACAO 4  // medições de 1s cada

// --- EEPROM ---
#define EEPROM_GIRO_MAX_HORARIO      0   // int32_t (4 bytes)
#define EEPROM_GIRO_MAX_ANTIHORARIO  4   // int32_t (4 bytes)
#define EEPROM_PID                   8   // 3 floats: kp, ki, kd (12 bytes)
#define EEPROM_FREIO_PD             20   // 2 floats: kpFreio, kdFreio (8 bytes)
#define EEPROM_FREIO_DELTA          28   // int32_t: deltaFreio (4 bytes)

bool calibracaoEmAndamento = false;

// --- PID (biblioteca PID_v1) ---
double Setpoint = 0, Input = 0, Output = 0;
PID myPID(&Input, &Output, &Setpoint, 2.8, 0.5, 5);

int32_t contadorGiroMaximoHorario = 0;
int32_t contadorGiroMaximoAntiHorario = 0;
bool encoderAtivo = false;

// --- Freio ---
#define MODO_FREIO_BREAK   0
#define MODO_FREIO_TRAVADO 1
float KP_FREIO_TRAVADO = 15.0;
float KD_FREIO_TRAVADO = 8.0;

int8_t modoFreio = MODO_FREIO_BREAK;
int32_t contadorHold = 0;
int32_t DELTA_FREIO_TRAVADO = 1;
bool deltaFreioCustom = false;  // true = valor definido pelo usuário, não recalcular
int32_t erroGiroAnterior = 0;

// --- Girar por ângulo ---
int16_t giroComando = 0;
int32_t contadorAbsolutoDesejado = 0;
bool comandoGirarPorAngulo = false;

// --- Timeout de segurança ---
#define TIMEOUT_COMUNICACAO 5000        // ms sem comando = para motor
#define TIMEOUT_RESET_WATCHDOG 10000    // ms sem comando = deixa o watchdog resetar
#define WDT_TIMEOUT WDTO_2S              // período do watchdog (2s)

// --- ID ---
uint8_t MEU_ID;

// --- Encoder (volatile — acessado na ISR) ---
volatile int16_t encoderCount = 0;
volatile uint8_t estadoEncoderAnterior = 0;

int32_t contadorAbsoluto = 0;
int8_t velocidadeMotor = 0;   // -100 a 100

uint8_t estadoMotor = 0;      // 0=parado, 1=horário, 2=anti-horário
uint8_t modoAtual = MODO_PID;

// --- Comunicação ---
uint32_t ultimaComunicacao = 0;

// =================================================================
// Tabela de quadratura para decoder do encoder
// =================================================================
static const int8_t tabelaQuadratura[] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

// =================================================================
// CRC
// =================================================================
uint8_t calcularCRC(const uint8_t* buf, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

// =================================================================
// Controle TX para half-duplex one-wire
// PA1 (TX) e PA2 (RX) ligados juntos no mesmo fio do barramento
// =================================================================
void habilitarTX() {
    PORTA.DIRSET = PIN1_bm;
    USART0.CTRLB |= USART_TXEN_bm;
}

void desabilitarTX() {
    USART0.CTRLB &= ~USART_TXEN_bm;
    PORTA.DIRCLR = PIN1_bm;
}

// =================================================================
// Encoder — decodificação em quadratura por ISR (PORTB)
// PB4 = canal A, PB5 = canal B
// =================================================================
ISR(PORTB_PORT_vect) {
    uint8_t flags = PORTB.INTFLAGS;
    PORTB.INTFLAGS = flags;  // Limpa todas as flags de interrupção

    // Só processa se foi PB4 ou PB5
    if (!(flags & (PIN4_bm | PIN5_bm))) return;

    uint8_t estadoAtual = (PORTB.IN >> 4) & 0x03;  // [PB5:PB4]
    uint8_t idx = (estadoEncoderAnterior << 2) | estadoAtual;
    int8_t delta = tabelaQuadratura[idx];

    encoderCount += delta;
    estadoEncoderAnterior = estadoAtual;
}

void setupEncoder() {
    noInterrupts();
    // PB4 e PB5 como entrada com pull-up, interrupção em ambas as bordas
    PORTB.DIRCLR = PIN5_bm | PIN4_bm;
    PORTB.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    PORTB.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    // Lê estado inicial para evitar contagem espúria no primeiro pulso
    estadoEncoderAnterior = (PORTB.IN >> 4) & 0x03;
    interrupts();
}

// =================================================================
// Controle do Motor
// =================================================================
void pararMotor() {
    digitalWrite(PINO1_MOTOR, LOW);
    digitalWrite(PINO2_MOTOR, LOW);
}

// PWM com sinal: positivo = frente (horário), negativo = trás (anti-horário)
void setMotorPWMSpeed(int16_t pwm) {
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;
    if (pwm > 0) {
        analogWrite(PINO1_MOTOR, pwm);
        digitalWrite(PINO2_MOTOR, LOW);
    } else if (pwm < 0) {
        digitalWrite(PINO1_MOTOR, LOW);
        analogWrite(PINO2_MOTOR, -pwm);
    } else {
        pararMotor();
    }
}

// Velocidade -100..100 mapeada diretamente para PWM 0..255
void setMotorSpeed(int8_t speed) {
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    if (speed > 0) {
        int16_t pwm = map(speed, 0, 100, 0, 255);
        setMotorPWMSpeed(pwm);
    } else if (speed < 0) {
        int16_t pwm = map(-speed, 0, 100, 0, 255);
        setMotorPWMSpeed(-pwm);
    } else {
        pararMotor();
    }
}

// =================================================================
// Delay que alimenta o watchdog — necessário para os delay() longos
// da calibração não serem interrompidos por um reset do WDT
// =================================================================
void delayWDT(uint32_t ms) {
    while (ms > 0) {
        uint16_t bloco = (ms > 250) ? 250 : ms;
        delay(bloco);
        wdt_reset();
        ms -= bloco;
    }
}

// =================================================================
// Calibração de velocidade máxima
// =================================================================
void calibrarVelocidadeMaxima() {
    calibracaoEmAndamento = true;

    // --- Sentido HORÁRIO (frente) ---
    setMotorSpeed(100);
    delayWDT(TEMPO_ESPERA_PARAR);        // Warm-up: motor estabiliza
    noInterrupts();
    encoderCount = 0;
    interrupts();
    for (int i = 0; i < QUANTIDADE_REPETICAO_CALIBRACAO; i++) {
        delayWDT(1000);
    }
    noInterrupts();
    int32_t contagemHorario = encoderCount;
    interrupts();
    pararMotor();

    // Média por segundo → por período PID → 80% do máximo
    contagemHorario /= QUANTIDADE_REPETICAO_CALIBRACAO;
    contagemHorario /= FREQUENCIA_PID;
    contagemHorario = (contagemHorario * 80) / 100;
    EEPROM.put(EEPROM_GIRO_MAX_HORARIO, contagemHorario);

    delayWDT(TEMPO_ESPERA_PARAR);

    // --- Sentido ANTI-HORÁRIO (trás) ---
    setMotorSpeed(-100);
    delayWDT(TEMPO_ESPERA_PARAR);        // Warm-up: motor estabiliza
    noInterrupts();
    encoderCount = 0;
    interrupts();
    for (int i = 0; i < QUANTIDADE_REPETICAO_CALIBRACAO; i++) {
        delayWDT(1000);
    }
    noInterrupts();
    int32_t contagemAntiHorario = encoderCount;
    interrupts();
    pararMotor();

    // Média por segundo → por período PID → 80% do máximo
    contagemAntiHorario /= QUANTIDADE_REPETICAO_CALIBRACAO;
    contagemAntiHorario /= FREQUENCIA_PID;
    contagemAntiHorario = (contagemAntiHorario * 80) / 100;
    EEPROM.put(EEPROM_GIRO_MAX_ANTIHORARIO, contagemAntiHorario);

    // Atualiza variáveis em memória
    contadorGiroMaximoHorario = contagemHorario;
    contadorGiroMaximoAntiHorario = contagemAntiHorario;
    if (!deltaFreioCustom) {
        DELTA_FREIO_TRAVADO = abs(contadorGiroMaximoHorario) / 4;
        if (DELTA_FREIO_TRAVADO < 1) DELTA_FREIO_TRAVADO = 1;
    }

    // Reseta estado
    noInterrupts();
    encoderCount = 0;
    interrupts();
    contadorAbsoluto = 0;
    myPID.Clear();
    velocidadeMotor = 0;
    Setpoint = 0;
    encoderAtivo = true;
    calibracaoEmAndamento = false;

    // Envia resposta com dados da calibração
    //   [3..4] = giroMaxHorario (big-endian, int16)
    //   [5..6] = giroMaxAntiHorario (big-endian, int16)
    uint8_t resp[TAMANHO_PACOTE_RESP];
    resp[0] = MEU_ID;
    resp[1] = MODO_CALIBRACAO;
    resp[2] = 0;
    resp[3] = (contadorGiroMaximoHorario >> 8) & 0xFF;
    resp[4] = contadorGiroMaximoHorario & 0xFF;
    resp[5] = (contadorGiroMaximoAntiHorario >> 8) & 0xFF;
    resp[6] = contadorGiroMaximoAntiHorario & 0xFF;
    resp[7] = calcularCRC(resp, 7);

    habilitarTX();
    Serial.write(resp, TAMANHO_PACOTE_RESP);
    Serial.flush();
    desabilitarTX();
    while (Serial.available()) Serial.read();
}

// =================================================================
// EEPROM — PID
// =================================================================
void salvaPIDnaEEPROM(float kp, float ki, float kd) {
    EEPROM.put(EEPROM_PID, kp);
    EEPROM.put(EEPROM_PID + sizeof(float), ki);
    EEPROM.put(EEPROM_PID + 2 * sizeof(float), kd);
}

void carregaPIDdaEEPROM() {
    float kp, ki, kd;
    EEPROM.get(EEPROM_PID, kp);
    EEPROM.get(EEPROM_PID + sizeof(float), ki);
    EEPROM.get(EEPROM_PID + 2 * sizeof(float), kd);
    if (!isnan(kp) && !isnan(ki) && !isnan(kd)) {
        myPID.SetTunings(kp, ki, kd);
    }
}

void salvaFreioPDnaEEPROM(float kpF, float kdF) {
    EEPROM.put(EEPROM_FREIO_PD, kpF);
    EEPROM.put(EEPROM_FREIO_PD + sizeof(float), kdF);
}

void carregaFreioPDdaEEPROM() {
    float kpF, kdF;
    EEPROM.get(EEPROM_FREIO_PD, kpF);
    EEPROM.get(EEPROM_FREIO_PD + sizeof(float), kdF);
    if (!isnan(kpF)) KP_FREIO_TRAVADO = kpF;
    if (!isnan(kdF)) KD_FREIO_TRAVADO = kdF;

    int32_t delta;
    EEPROM.get(EEPROM_FREIO_DELTA, delta);
    if (delta > 0 && delta != (int32_t)0xFFFFFFFF) {
        DELTA_FREIO_TRAVADO = delta;
        deltaFreioCustom = true;
    }
}

void carregarCalibracao() {
    EEPROM.get(EEPROM_GIRO_MAX_HORARIO, contadorGiroMaximoHorario);
    EEPROM.get(EEPROM_GIRO_MAX_ANTIHORARIO, contadorGiroMaximoAntiHorario);

    // Valida: se parecer EEPROM virgem (0xFFFFFFFF = -1) ou zero, desativa
    if (contadorGiroMaximoHorario != 0 && contadorGiroMaximoHorario != -1) {
        encoderAtivo = true;
    } else {
        contadorGiroMaximoHorario = 0;
        contadorGiroMaximoAntiHorario = 0;
        encoderAtivo = false;
    }

    DELTA_FREIO_TRAVADO = abs(contadorGiroMaximoHorario) / 4;
    if (DELTA_FREIO_TRAVADO < 1) DELTA_FREIO_TRAVADO = 1;

    carregaPIDdaEEPROM();
    carregaFreioPDdaEEPROM();  // pode sobrescrever DELTA se houver valor custom na EEPROM
}

// =================================================================
// Reset — volta todos os parâmetros ao estado inicial (pós-setup)
// =================================================================
void resetEquipamento() {
    // Para o motor
    pararMotor();

    // Reseta encoder
    noInterrupts();
    encoderCount = 0;
    interrupts();

    // Reseta contadores e estado
    contadorAbsoluto = 0;
    velocidadeMotor = 0;
    estadoMotor = 0;
    modoAtual = MODO_PID;

    // Reseta PID
    Setpoint = 0;
    Input = 0;
    Output = 0;
    myPID.Clear();

    // Reseta frenagem
    modoFreio = MODO_FREIO_BREAK;
    contadorHold = 0;
    erroGiroAnterior = 0;

    // Reseta giro por ângulo
    giroComando = 0;
    contadorAbsolutoDesejado = 0;
    comandoGirarPorAngulo = false;

    // Reseta calibração
    calibracaoEmAndamento = false;

    // Recarrega calibração e PID da EEPROM
    carregarCalibracao();

    // Reseta timeout
    ultimaComunicacao = millis();
}

// =================================================================
// Protocolo — Enviar resposta padrão
// =================================================================
void enviarResposta() {
    uint8_t resp[TAMANHO_PACOTE_RESP];
    resp[0] = MEU_ID;
    resp[1] = modoAtual;
    resp[2] = estadoMotor;
    resp[3] = (contadorAbsoluto >> 24) & 0xFF;  // MSB
    resp[4] = (contadorAbsoluto >> 16) & 0xFF;
    resp[5] = (contadorAbsoluto >> 8)  & 0xFF;
    resp[6] = contadorAbsoluto & 0xFF;          // LSB
    resp[7] = calcularCRC(resp, 7);

    habilitarTX();
    Serial.write(resp, TAMANHO_PACOTE_RESP);
    Serial.flush();
    desabilitarTX();
    while (Serial.available()) Serial.read();  // Descarta eco
}

// =================================================================
// Protocolo — Processar pacote recebido
// =================================================================
bool processarPacote(const uint8_t* buf) {
    if ((buf[0] != MEU_ID) || (calcularCRC(buf, 5) != buf[5])){ //falha
        Serial.flush();
        desabilitarTX();
        while (Serial.available()) Serial.read();  // Descarta eco
        return false;
    }


    uint8_t modo = buf[1];

    // --- Reset geral ---
    if (modo == MODO_RESET) {
        resetEquipamento();
        enviarResposta();
        return true;
    }

    // --- Calibração: executa e retorna (bloqueante) ---
    if (modo == MODO_CALIBRACAO) {
        modoAtual = MODO_CALIBRACAO;
        calibrarVelocidadeMaxima();
        return true;
    }

    // --- Ler calibração atual ---
    if (modo == MODO_GET_CALIBRACAO) {
        uint8_t resp[TAMANHO_PACOTE_RESP];
        resp[0] = MEU_ID;
        resp[1] = MODO_GET_CALIBRACAO;
        resp[2] = encoderAtivo ? 1 : 0;
        resp[3] = (contadorGiroMaximoHorario >> 8) & 0xFF;
        resp[4] = contadorGiroMaximoHorario & 0xFF;
        resp[5] = (contadorGiroMaximoAntiHorario >> 8) & 0xFF;
        resp[6] = contadorGiroMaximoAntiHorario & 0xFF;
        resp[7] = calcularCRC(resp, 7);

        habilitarTX();
        Serial.write(resp, TAMANHO_PACOTE_RESP);
        Serial.flush();
        desabilitarTX();
        while (Serial.available()) Serial.read();
        return true;
    }

    // --- Forçar calibração: envia valores máximos manualmente ---
    if (modo == MODO_SET_CALIBRACAO) {
        uint8_t direcao = buf[2];  // 0 = horário, 1 = anti-horário
        int16_t valor = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);

        if (direcao == 0) {
            contadorGiroMaximoHorario = valor;
            EEPROM.put(EEPROM_GIRO_MAX_HORARIO, contadorGiroMaximoHorario);
        } else {
            contadorGiroMaximoAntiHorario = valor;
            EEPROM.put(EEPROM_GIRO_MAX_ANTIHORARIO, contadorGiroMaximoAntiHorario);
        }

        if (!deltaFreioCustom) {
            DELTA_FREIO_TRAVADO = abs(contadorGiroMaximoHorario) / 4;
            if (DELTA_FREIO_TRAVADO < 1) DELTA_FREIO_TRAVADO = 1;
        }

        // Ativa encoder se ambos os valores foram definidos
        if (contadorGiroMaximoHorario != 0 && contadorGiroMaximoAntiHorario != 0) {
            encoderAtivo = true;
        }

        // Resposta com valores atuais da calibração
        uint8_t resp[TAMANHO_PACOTE_RESP];
        resp[0] = MEU_ID;
        resp[1] = MODO_SET_CALIBRACAO;
        resp[2] = direcao;
        resp[3] = (contadorGiroMaximoHorario >> 8) & 0xFF;
        resp[4] = contadorGiroMaximoHorario & 0xFF;
        resp[5] = (contadorGiroMaximoAntiHorario >> 8) & 0xFF;
        resp[6] = contadorGiroMaximoAntiHorario & 0xFF;
        resp[7] = calcularCRC(resp, 7);

        habilitarTX();
        Serial.write(resp, TAMANHO_PACOTE_RESP);
        Serial.flush();
        desabilitarTX();
        while (Serial.available()) Serial.read();
        return true;
    }

    // --- Ajuste de constantes PID ---
    if (modo >= MODO_SET_KP && modo <= MODO_SET_KD) {
        uint16_t valorRaw = ((uint16_t)buf[2] << 8) | buf[3];
        float novoK = (float)valorRaw / 100.0f;

        // Carrega valores atuais para preservar os que não mudam
        float kp, ki, kd;
        EEPROM.get(EEPROM_PID, kp);
        EEPROM.get(EEPROM_PID + sizeof(float), ki);
        EEPROM.get(EEPROM_PID + 2 * sizeof(float), kd);
        if (isnan(kp)) kp = 2.8;
        if (isnan(ki)) ki = 0.5;
        if (isnan(kd)) kd = 5.0;

        switch (modo) {
            case MODO_SET_KP: kp = novoK; break;
            case MODO_SET_KI: ki = novoK; break;
            case MODO_SET_KD: kd = novoK; break;
        }
        myPID.SetTunings(kp, ki, kd);
        salvaPIDnaEEPROM(kp, ki, kd);

        // Resposta: ecoa modo e valor confirmado
        uint8_t resp[TAMANHO_PACOTE_RESP];
        resp[0] = MEU_ID;
        resp[1] = modo;
        resp[2] = 0;
        resp[3] = (valorRaw >> 8) & 0xFF;
        resp[4] = valorRaw & 0xFF;
        resp[5] = 0;
        resp[6] = 0;
        resp[7] = calcularCRC(resp, 7);

        habilitarTX();
        Serial.write(resp, TAMANHO_PACOTE_RESP);
        Serial.flush();
        desabilitarTX();
        while (Serial.available()) Serial.read();
        return true;
    }

    // --- Ajuste de DELTA do freio travado ---
    if (modo == MODO_SET_DELTA_FREIO) {
        uint16_t valorRaw = ((uint16_t)buf[2] << 8) | buf[3];
        DELTA_FREIO_TRAVADO = (int32_t)valorRaw;
        if (DELTA_FREIO_TRAVADO < 1) DELTA_FREIO_TRAVADO = 1;
        deltaFreioCustom = true;
        EEPROM.put(EEPROM_FREIO_DELTA, DELTA_FREIO_TRAVADO);

        uint8_t resp[TAMANHO_PACOTE_RESP];
        resp[0] = MEU_ID;
        resp[1] = modo;
        resp[2] = 0;
        resp[3] = (valorRaw >> 8) & 0xFF;
        resp[4] = valorRaw & 0xFF;
        resp[5] = 0;
        resp[6] = 0;
        resp[7] = calcularCRC(resp, 7);

        habilitarTX();
        Serial.write(resp, TAMANHO_PACOTE_RESP);
        Serial.flush();
        desabilitarTX();
        while (Serial.available()) Serial.read();
        return true;
    }

    // --- Ajuste de constantes PD do freio travado ---
    if (modo == MODO_SET_KP_FREIO || modo == MODO_SET_KD_FREIO) {
        uint16_t valorRaw = ((uint16_t)buf[2] << 8) | buf[3];
        float novoK = (float)valorRaw / 100.0f;

        if (modo == MODO_SET_KP_FREIO) {
            KP_FREIO_TRAVADO = novoK;
        } else {
            KD_FREIO_TRAVADO = novoK;
        }
        salvaFreioPDnaEEPROM(KP_FREIO_TRAVADO, KD_FREIO_TRAVADO);

        uint8_t resp[TAMANHO_PACOTE_RESP];
        resp[0] = MEU_ID;
        resp[1] = modo;
        resp[2] = 0;
        resp[3] = (valorRaw >> 8) & 0xFF;
        resp[4] = valorRaw & 0xFF;
        resp[5] = 0;
        resp[6] = 0;
        resp[7] = calcularCRC(resp, 7);

        habilitarTX();
        Serial.write(resp, TAMANHO_PACOTE_RESP);
        Serial.flush();
        desabilitarTX();
        while (Serial.available()) Serial.read();
        return true;
    }

    // --- Configura modo de frenagem ---
    if (modo == MODO_SET_FREIO) {
        modoFreio = buf[2];
        enviarResposta();
        return true;
    }

    // --- Apenas lê estado atual ---
    if (modo == MODO_ATUALIZA) {
        enviarResposta();
        return true;
    }

    // --- MODO_PID ou MODO_PWM ---
    modoAtual = modo;
    velocidadeMotor = (int8_t)buf[2];
    uint16_t giro = ((uint16_t)buf[3] << 8) | buf[4];

    if (modo == MODO_PID) {
        // Ativa encoder para PID se calibrado
        if (contadorGiroMaximoHorario != 0) {
            encoderAtivo = true;
            // Mapeia velocidade (-100..100) para setpoint em pulsos/período PID
            if (velocidadeMotor >= 0) {
                Setpoint = map(velocidadeMotor, 0, 100, 0, contadorGiroMaximoHorario);
            } else {
                Setpoint = map(velocidadeMotor, 0, -100, 0, contadorGiroMaximoAntiHorario);
            }
        } else {
            // Sem calibração: fallback para PWM direto
            encoderAtivo = false;
            setMotorSpeed(velocidadeMotor);
        }

        // Se velocidade zero, acumula encoder e salva posição de hold
        if (velocidadeMotor == 0) {
            noInterrupts();
            int16_t enc = encoderCount;
            encoderCount = 0;
            interrupts();
            contadorAbsoluto += enc;
            contadorHold = contadorAbsoluto;
            erroGiroAnterior = 0;
        }
    } else if (modo == MODO_PWM) {
        // Modo potência: PID desativado, mas encoder continua contando
        if (velocidadeMotor == 0) {
            encoderAtivo = true;
            Setpoint = 0;
        } else {
            encoderAtivo = false;
        }
        setMotorSpeed(velocidadeMotor);

        // Acumula encoder pendente
        noInterrupts();
        int16_t enc = encoderCount;
        encoderCount = 0;
        interrupts();
        contadorAbsoluto += enc;
        contadorAbsolutoDesejado = contadorAbsoluto;
        contadorHold = contadorAbsoluto;
        erroGiroAnterior = 0;
    }

    // Configura giro por ângulo (apenas no modo PID)
    if (giro > 0 && modo == MODO_PID) {
        noInterrupts();
        int16_t pendingEnc = encoderCount;
        interrupts();
        if (Setpoint >= 0) {
            contadorAbsolutoDesejado = contadorAbsoluto + giro + pendingEnc;
        } else {
            contadorAbsolutoDesejado = contadorAbsoluto - giro + pendingEnc;
        }
        comandoGirarPorAngulo = true;
    } else {
        comandoGirarPorAngulo = false;
    }

    // Se velocidade zero, desativa giro por ângulo
    if (velocidadeMotor == 0) {
        comandoGirarPorAngulo = false;
    }

    // Atualiza estado imediato do motor
    if (velocidadeMotor > 0) {
        estadoMotor = 1;  // horário
    } else if (velocidadeMotor < 0) {
        estadoMotor = 2;  // anti-horário
    } else {
        estadoMotor = 0;  // parado
    }

    enviarResposta();
    return true;
}

// =================================================================
// Setup
// =================================================================
void setup() {
    pinMode(PINO1_MOTOR, OUTPUT);
    pinMode(PINO2_MOTOR, OUTPUT);
    pinMode(LED, OUTPUT);
    pararMotor();

    // Prescaler do TCA0 (split mode) para frequência do PWM
    TCA0.SPLIT.CTRLA = PWM_PRESCALER | TCA_SPLIT_ENABLE_bm;

    setupEncoder();

    MEU_ID = SIGROW.SERNUM9;

    carregarCalibracao();

    Serial.swap(1);        // Pinos alternativos: PA1 (TX) e PA2 (RX)
    Serial.begin(250000);
    desabilitarTX();
    delay(10);
    while (Serial.available()) Serial.read();
    for(uint8_t i = 0; i < 3; i++) {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
    }
    ultimaComunicacao = millis();

    wdt_reset();
    wdt_enable(WDT_TIMEOUT);  // Watchdog de 2s: reseta se o loop travar
}

// =================================================================
// Loop
// =================================================================
void loop() {
    // --- Recepção de pacotes ---
    static uint8_t buffer[TAMANHO_PACOTE_CMD];
    static uint8_t idx = 0;
    static uint32_t ultimoByteMs = 0;
    static uint32_t ultimoByteUs = 0;

    while (Serial.available()) {
        buffer[idx] = Serial.read();
        ultimoByteMs = millis();
        ultimoByteUs = micros();
        idx++;
        if (idx >= TAMANHO_PACOTE_CMD) {
            digitalWrite(LED, HIGH);
            if(processarPacote(buffer)) { //só considero se processarPacote retornar true (pacote válido e destinado a mim)
                ultimaComunicacao = millis();
            }
            idx = 0;
        }
        //delayMicroseconds(300); 
    }

    if (idx > 0 && (micros() - ultimoByteUs) > TIMEOUT_PACOTE) {
        idx = 0;
    }

    // --- Watchdog: só alimenta se houve comunicação serial recente.
    // Sem comando por TIMEOUT_RESET_WATCHDOG, o WDT deixa de ser resetado
    // e o próprio watchdog reinicia o equipamento em até WDT_TIMEOUT.
    if ((millis() - ultimaComunicacao) < TIMEOUT_RESET_WATCHDOG) {
        wdt_reset();
    }

    // --- Calibração em andamento: não faz nada no loop ---
    if (calibracaoEmAndamento) return;

    // --- Timeout de segurança ---
    if ((millis() - ultimaComunicacao) > TIMEOUT_COMUNICACAO) {
        velocidadeMotor = 0;
        Setpoint = 0;
        pararMotor();
        myPID.Clear();
        estadoMotor = 0;
        comandoGirarPorAngulo = false;
        // Acumula encoder para evitar overflow/perda
        noInterrupts();
        contadorAbsoluto += encoderCount;
        encoderCount = 0;
        interrupts();
        digitalWrite(LED, LOW);
        return;
    }

    // --- Controle PID (quando encoder ativo) ---
    if (encoderAtivo) {
        if (Setpoint != 0) {
            // Lê encoder atomicamente para o PID
            noInterrupts();
            Input = encoderCount;
            interrupts();

            if (myPID.Compute()) {
                // Acumula encoder no contador absoluto
                noInterrupts();
                int16_t enc = encoderCount;
                encoderCount = 0;
                interrupts();
                contadorAbsoluto += enc;

                // Aplica saída PID ao motor conforme direção do setpoint
                if (Setpoint >= 0) {
                    setMotorPWMSpeed((int16_t)Output);
                } else {
                    setMotorPWMSpeed(-1 * (int16_t)Output);
                }

                // Atualiza estado do motor baseado na direção real
                if (Output > 0) {
                    if (Setpoint > 0) estadoMotor = 1;       // horário
                    else              estadoMotor = 2;        // anti-horário
                } else {
                    estadoMotor = 0;
                }

                contadorHold = contadorAbsoluto;
            }
        } else {
            // Velocidade zero: aplica modo de frenagem
            myPID.Clear();

            noInterrupts();
            int16_t enc = encoderCount;
            encoderCount = 0;
            interrupts();
            contadorAbsoluto += enc;

            if (modoFreio == MODO_FREIO_BREAK) {
                setMotorPWMSpeed(0);
                estadoMotor = 0;
            } else if (modoFreio == MODO_FREIO_TRAVADO) {
                int32_t erroGiro = contadorAbsoluto - contadorHold;
                int32_t derivativo = erroGiro - erroGiroAnterior;
                erroGiroAnterior = erroGiro;

                int16_t potencia = abs(KP_FREIO_TRAVADO * erroGiro + KD_FREIO_TRAVADO * derivativo);
                if (potencia > 255) potencia = 255;

                if (erroGiro < -DELTA_FREIO_TRAVADO) {
                    setMotorPWMSpeed(potencia);   // Corrige para frente
                    estadoMotor = 1;
                } else if (erroGiro > DELTA_FREIO_TRAVADO) {
                    setMotorPWMSpeed(-potencia);  // Corrige para trás
                    estadoMotor = 2;
                } else {
                    setMotorPWMSpeed(0);
                    estadoMotor = 0;
                }
            }
        }

        // --- Verificação do giro por ângulo ---
        if (comandoGirarPorAngulo) {
            if (Setpoint > 0) {
                if (contadorAbsoluto >= contadorAbsolutoDesejado) {
                    Setpoint = 0;
                    comandoGirarPorAngulo = false;
                    contadorHold = contadorAbsoluto;
                    erroGiroAnterior = 0;
                }
            } else if (Setpoint < 0) {
                if (contadorAbsoluto <= contadorAbsolutoDesejado) {
                    Setpoint = 0;
                    comandoGirarPorAngulo = false;
                    contadorHold = contadorAbsoluto;
                    erroGiroAnterior = 0;
                }
            }
        }
    }

    digitalWrite(LED, LOW);
}