#include "Config.h"
#include <math.h>

const float L1 = 238.0; 
const float L4 = 238.0;
const float L5 = 66.0;
const float L3 = 238.0;
const float L2 = 68.0;

void calculateAngles(float x, float y, float z, float &angleBase, float &angle1, float &angle2) {
    float thetaBase = atan2(y, x);
    float disSq = x*x + y*y + z*z;
    float dis = sqrt(disSq);
    float dis2D = sqrt(pow(x,2)+pow(y,2));
    float theta0 = atan2(z,dis2D);
    
    float cos_theta1 = (L1 * L1 + disSq - L4 * L4) / (2 * L1 * dis);
    float theta1 = acos(cos_theta1); 
    
    float cos_theta2 = (L4 * L4 + disSq - L1 * L1) / (2 * L4 * dis);
    float theta2 = acos(cos_theta2); 

    float L_sum = L4 + L5;
    float s3 = sqrt(disSq + L_sum * L_sum - 2 * dis * L_sum * cos(theta2));
    float cos_theta3 = (L3 * L3 + s3 * s3 - L2 * L2) / (2 * L3 * s3);
    float theta3 = acos(cos_theta3); 
    
    float cos_theta4 = (s3 * s3 + L2 * L2 - L3 * L3) / (2 * s3 * L2);
    float theta4 = acos(cos_theta4); 
    
    angleBase = thetaBase * 180.0 / M_PI; 
    angle1 = (theta0 + theta1) * 180.0 / M_PI;
    angle2 = (theta0 + theta1 + theta3 + theta4) * 180.0 / M_PI; 
}