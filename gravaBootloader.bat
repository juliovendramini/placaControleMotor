@echo off
echo ============================================
echo  GRAVANDO BOOTLOADER OPTIBOOT NO ATtiny1616
echo ============================================
echo.
echo ATENCAO: Apos gravar, o pino PA0 sera configurado como RESET.
echo          UPDI sera DESABILITADO permanentemente!
echo          Futuras gravacoes so via Serial (Optiboot).
echo          Para restaurar UPDI, necessario programador HV UPDI.
echo.
pause

REM Fuses:
REM   fuse0 (WDTCFG)  = 0x00 (watchdog desabilitado)
REM   fuse2 (OSCCFG)   = 0x02 (oscilador 20MHz)
REM   fuse4 (TCD0CFG)  = 0x00
REM   fuse6 (SYSCFG1)  = 0x04 (SUT = 32ms)
REM   fuse7 (APPEND)   = 0x00
REM   fuse8 (BOOTEND)  = 0x02 (512 bytes para Optiboot)
REM   fuse5 (SYSCFG0)  = 0b11001000 = 0xC8 (RSTPINCFG=Reset, EESAVE=off, CRC=off)
REM
REM Bootloader: optiboot_txyz_alt_extr.hex
REM   _alt  = UART alternativo (PA1=TX, PA2=RX) - compativel com Serial.swap(1)
REM   _extr = entra no bootloader por reset externo, timeout 1 segundo
REM
REM IMPORTANTE: fuse5 e gravado POR ULTIMO pois desabilita UPDI ao mudar PA0 para Reset

C:\Users\Julio\AppData\Local\Arduino15\packages\DxCore\tools\avrdude\6.3.0-arduino17or18\bin\avrdude -CC:\Users\Julio\AppData\Local\Arduino15\packages\megaTinyCore\hardware\megaavr\2.6.10\avrdude.conf -v -pattiny1616 -cjtag2updi -PCOM8 -b115200 -Ufuse0:w:0x00:m -Ufuse2:w:0x02:m -Ufuse4:w:0x00:m -Ufuse6:w:0x04:m -Ufuse7:w:0x00:m -Ufuse8:w:0x02:m -Uflash:w:C:\Users\Julio\AppData\Local\Arduino15\packages\megaTinyCore\hardware\megaavr\2.6.10\bootloaders\hex\optiboot_txyz_alt_extr.hex:i -Ufuse5:w:0b11001000:m

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERRO ao gravar bootloader!
    pause
    exit /b 1
)

echo.
echo ============================================
echo  BOOTLOADER GRAVADO COM SUCESSO!
echo ============================================
echo.
echo Proximos passos:
echo   1. Na Arduino IDE, selecione:
echo      Board: ATtiny3216/1616/... w/Optiboot
echo      Chip: ATtiny1616
echo      Clock: 20 MHz internal
echo      Bootloader UART: Alternate Serial
echo      Reset pin: Reset (default entry condition)
echo   2. Conecte um adaptador USB-Serial:
echo      TX do adaptador -^> PA2 (RX do ATtiny)
echo      RX do adaptador -^> PA1 (TX do ATtiny)
echo      DTR -^> capacitor 100nF -^> PA0 (Reset) para auto-reset
echo   3. Use "Upload" na Arduino IDE normalmente
echo.
pause
