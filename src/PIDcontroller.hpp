#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "rover-controller/feedback.msg"
#include <vector>

template <typename T>
PID::PID(std::vector<T> Kp, std::vector<T> Ki, std::vector<T> Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->previous_error = std::vector<T>(Kp.size(), 0);
    this->integral = std::vector<T>(Kp.size(), 0);
    this->last_time = ros::Time();
}

template <typename T>
std::vector<T> PID::calculate(std::vector<T> target, std::vector<T> measured, ros::Time current_time) {
    if (last_time.isZero()) {
        last_time = current_time; // Set last_time in the first calculate call
    }
    ros::Duration dt = current_time - last_time;
    dt = 1 / dt.toSec();
    int s = target.size();
    std::vector<T> error(s);
    std::vector<T> derivative(s);
    std::vector<T> output(s);

    for (int i = 0; i < target.size(); i++) {
        error[i] = target[i] - measured[i];
        integral[i] += error[i] * dt; 
        derivative[i] = (error[i] - previous_error[i]) * dt;
        output[i] = Kp[i] * error[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];
        previous_error[i] = error[i];
    }
    last_time = current_time;
    return output;
}