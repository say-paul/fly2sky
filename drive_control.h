#include <Servo.h>
#include "config.h"

class MotorControl {
    public:
        MotorControl(int st[],int sv[],int angle) {
            for(int i=0;i<2;i++) {
                this->stepper[i].attach(st[i]);
            }
            for(int i=0;i<2;i++) {
                this->servo[i].attach(st[i],MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
                servo[i].write(0);
            }
            this->default_angle = angle;
        }
        void reset_angle() {
             stepper[0].write(default_angle);
             stepper[1].write(default_angle);
        }
        void forward(int angle) {
             stepper[0].write(default_angle + angle);
             stepper[1].write(default_angle - angle);
        }
        void backward(int angle) {
             stepper[0].write(default_angle - angle);
             stepper[1].write(default_angle + angle);
        }
        void yaw_right(float angle);
        void yaw_left(float angle);
        void roll_right(float angle);
        void roll_left(float angle);
        void up(float thrust);
        void down(float thrust);
        void hold_steady();
        void takeoff();
        void land();
        void start();
        /* swapping is avilabale for bicopters only */
        void swap_stepper() {
            std::swap(stepper[0],stepper[1]);
        }
        void swap_servo() {
            std::swap(servo[0],servo[1]);
        }


    private:
        Servo stepper[2];
        Servo servo[2];
        int default_angle;
};