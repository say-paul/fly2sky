#include "drive_control.h"

MotorControl::MotorControl(int st[],int sv[],int angle,int min,int max) {
            for(int i=0;i<2;i++) {
                this->stepper[i].attach(st[i]);
            }
            for(int i=0;i<2;i++) {
                this->servo[i].attach(st[i],min,max);
                servo[i].write(0);
            }
            this->default_angle = angle;
        }

void MotorControl::reset_angle() {
             stepper[0].write(default_angle);
             stepper[1].write(default_angle);
        }

void MotorControl::forward(int angle) {
             stepper[0].write(default_angle + angle);
             stepper[1].write(default_angle - angle);
        }
void MotorControl::backward(int angle) {
             stepper[0].write(default_angle - angle);
             stepper[1].write(default_angle + angle);
        }

void MotorControl::swap_stepper() {
            std::swap(stepper[0],stepper[1]);
        }

void MotorControl::swap_servo() {
            std::swap(servo[0],servo[1]);
        }