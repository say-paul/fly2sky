#include <Servo.h>


class MotorControl {
    public:
        MotorControl(int st[],int sv[],int angle,int min,int max);
        void reset_angle();
        void forward(int angle);
        void backward(int angle);
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
        void swap_stepper();
        void swap_servo();


    private:
        Servo stepper[2];
        Servo servo[2];
        int default_angle;
};