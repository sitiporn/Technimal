#include "PIDControl.h"

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

PID::PID(double* input,double* output,double* setpoint,double kp,double ki,double kd)
{
    _input = input;
    _output= output;
    _setPoint = setpoint;
    _Kp = kp;
    _Kd = kd;
    _Ki = ki;
}
bool PID::Initialize()
{
   //_prev_time = Time;
    double SampleTimeInSec = ((double)SampleTime)/1000;
   _Kp = _Kp;
   _Ki = _Ki * SampleTimeInSec;
   _Kd = _Kd / SampleTimeInSec;
   _I = 0;
   _cumError =0; //Integral 
   _prevError =0;// _setPoint-_input; //prevErr
   _prevInput = 0;
   _previousTime = millis()-SampleTime;
   drift_=0;
    
 if(*_setPoint>=90)
 {    
      double SP = *_setPoint;
      drift_  = this->Map(SP,90,100,0.2,1.0);
      SP = this->Map(SP,90,100,88,89.75);
      *_setPoint = SP;
  }
  else if(*_setPoint>=86 && *_setPoint<=89)
  {    
        int SP = (int)*_setPoint;
      
        switch (SP)
        {
          case 86:  *_setPoint = 85.75;
                break;
          case 87: *_setPoint = 86;
                   drift_=0.1;
                break;
          case 88: *_setPoint = 87;
                    drift_=0.1;
                break;
          case 89: *_setPoint = 87.5;
                    drift_=1.5;
                break;
          default:
              break;
        }
  }
  
   
}
void PID::Compute()
{ 
  _currentTime = millis();
  _dt = (double)(_currentTime-_previousTime);
 
  double input = *_input;
  double output = *_output;

  if(_dt >=SampleTime)
  {   _error = *_setPoint-input;
      _I += (_Ki*_error);
  //_rateError = (_error - _prevError);//_dt;
     double dInput = (input - _prevInput); 

     output =_Kp*_error+_I-_Kd*dInput; 
      //_output +=12;
     //_output = Kp*_error +_outCumErr-Kd*_rateError;
      
      if(output>Max)
            output = Max;
      else if(output<Min)
            output = Min;
      
      _prevInput = input;
      _previousTime = _currentTime;
      *_output = output;
  }
}
double PID::Map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
