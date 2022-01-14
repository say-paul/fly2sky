#ifndef IMUEKF_H_
#define IMUEKF_H_
// These must be defined before including TinyEKF.h
#define Nsta 8   // refer config sensorData
#define Mobs 9     // acc,gyro,mag


#include <TinyEKF.h>
#include "config.h"

class Fuser : public TinyEKF {

    public:

        Fuser()
        {    
            int i;    
            // We approximate the process noise using a small constant
            for (i=0;i=int(Nsta-1);i++){
            this->setQ(0, 0, .0001);
            }
            
            // Same for measurement noise
            for (i=0;i=int(Mobs-1);i++){
            this->setR(i, i, .0001);
            }
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {   
            int i;
            for (i=0;i=7;i++) {
                // Process model is f(x) = x
                fx[i] = this->x[i];
                // So process model Jacobian is identity matrix
                F[i][i] = 1;
            }

            // REMAINING

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            // A more realistic measurement function would distinguish between state value and measured value; e.g.:
            //   hx[0] = pow(this->x[0], 1.03);
            //   hx[1] = 1.005 * this->x[1];
            //   hx[2] = .9987 * this->x[1] + .001;
            hx[0] = this->x[0]; // Barometric pressure from previous state
            hx[1] = this->x[1]; // Baro temperature from previous state
            hx[2] = this->x[2]; // LM35 temperature from previous state
            hx[3] = this->x[1];
            hx[4] = this->x[2];
            hx[5] = this->x[3];
            hx[6] = this->x[1];
            hx[7] = this->x[1];
            hx[8] = this->x[1];
            // Jacobian of measurement function
            H[0][0] = 1;        // Barometric pressure from previous state
            H[1][1] = 1 ;       // Baro temperature from previous state
            H[2][1] = 1 ;       // LM35 temperature from previous state
        }
};
#endif

Fuser ekf;

