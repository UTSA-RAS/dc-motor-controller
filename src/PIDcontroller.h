#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <vector>

class PID {
public:
    template <typename T>
    PID(std::vector<T> Kp, std::vector<T> Ki, std::vector<T> Kd);
    template <typename T>
    std::vector<T> calculate(std::vector<T> target, std::vector<T> measured);

private:
    template <typename T>
    T Kp; // Proportional gain
    template <typename T>
    T Ki; // Integral gain
    template <typename T>
    T Kd; // Derivative gain
    
    template <typename T>
    T previous_error;
    template <typename T>
    T integral;

    ros::Time last_time;
};

#include "PIDcontroller.hpp"
#endif // PIDCONTROLLER_H