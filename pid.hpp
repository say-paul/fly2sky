float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// PID coefficients
float Kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll