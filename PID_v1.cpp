/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include <Arduino.h>

#include "PID_v1.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd)
{
   myOutput = Output;
   myInput = Input;
   mySetpoint = Setpoint;

   PID::SetOutputLimits(-255, 255);				//default output limit corresponds to
                                 //the arduino pwm limits
   this->SetTunings(Kp, Ki, Kd);		//we default to using the br3ttb
                                 //tuning formula (except with the I and D
                                 //parameters swapped)
   SampleTime = TEMPO_PID;
   lastTime = millis()-SampleTime;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myInput;
      double error = *mySetpoint - input;
      double dInput = (input - lastInput);
      if(*mySetpoint < 0){ //se for negativo inverto tudo
         error *= -1;
         dInput *= -1;
      }

      if(direcao == HORARIO && *mySetpoint < 0){
         outputSum = 0;
         direcao = ANTIHORARIO;
      }else if(direcao == ANTIHORARIO && *mySetpoint >= 0){
         direcao = HORARIO;
         outputSum = 0;
      }

      if(*mySetpoint == 0){
         outputSum = 0;
      }

      outputSum += (ki * error);
      if(outputSum > 250) outputSum = 250;
      else if(outputSum < -250) outputSum = -250;

      /*Compute PID Output*/
	   double output;
      double d;
      output = kp * error;
      if(*mySetpoint >= 0){
         output += *mySetpoint * kp;
      }else{
         output -= *mySetpoint * kp;
      }
      output += outputSum;
      d = kd * dInput;
      output -= d;

      
	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	    *myOutput = output;
      /*Serial.print(outputSum);
      Serial.print(" ");
      Serial.print(*mySetpoint);
      Serial.print(" ");
      Serial.print(*myInput);
      Serial.print(" ");
      Serial.println(*myOutput);
      */
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
   kp = Kp;
   ki = Ki;
   kd = Kd;

}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(*myOutput > outMax) *myOutput = outMax;
   else if(*myOutput < outMin) *myOutput = outMin;

   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;
}


/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

void PID::Clear()
{
   outputSum = 0;
   lastInput = 0;
   lastTime = millis();
   *myOutput = 0;
   direcao = HORARIO;

}
