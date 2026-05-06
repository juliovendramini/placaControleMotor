#ifndef PID_v1_h
#define PID_v1_h

#define FREQUENCIA_PID 40
#define TEMPO_PID 25 // Tempo em milissegundos entre as chamadas do PID


class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  
#define HORARIO 0
#define ANTIHORARIO 1

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double);     //   Setpoint.  Initial tuning parameters are also set here

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    
    void Clear();
	
  private:
	void Initialize();
	
	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
  uint8_t direcao;
};
#endif

