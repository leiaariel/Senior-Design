#ifndef MY_MOTOR_H
#define MY_MOTOR_H

#include <Arduino.h>

class Motor 
{
  private: 

  public: 
    Motor();
    void init();
    void supportOn(Potentiometer pot);
    void supportOff(Potentiometer pot);
    void setGap(float newGap);
    void moveBy(float ballScrewDistance); //maybe not float but int?
    int getPos(); //maybe not int but float?
    int getVelocity();
};

#endif
