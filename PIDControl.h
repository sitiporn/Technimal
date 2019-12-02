#ifndef _PIDControl_H
#define _PIDControl_H

#define Max 255
#define Min 0 
#define SampleTime 100

class PID
{

private:
    double* _input;
    double* _output;
    double* _setPoint;
    double _cumError;
    double _rateError;
    double _I;   //
    
    double _dt;
    double _error;
    double _prevError;
    double _prevInput;

    unsigned long 
          _currentTime,
          _previousTime,
          _prev_time;
     double _Kp;
     double _Ki;
     double _Kd;

public:
    double drift_;
    PID(double*,double*,double*,double =0.225,double=3.5,double=0.0025);
    ~PID(){};
    void Compute();
    bool Initialize();
    double Map(double x, double in_min, double in_max, double out_min, double out_max);

};

#endif