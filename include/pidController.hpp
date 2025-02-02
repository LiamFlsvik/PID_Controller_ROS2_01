
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>

class pidController{
    public:
    pidController(double kp_, double ki_, double kd_): kp(kp_), ki(ki_), kd(kd_), i(0), last_time(std::chrono::steady_clock::now()) {
        pid_update = [this](double reference, double measured_angle) -> double {
            
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time-last_time).count();
            last_time = current_time;
        
            error = reference-measured_angle;
            double p = kp*error;
            i += ki*error*dt;
            double d = kd*(error-last_error)/dt;
            last_error = error;
            
            return p+i+d;
        };
    }  

    double update(double reference, double measured_angle){
        return pid_update(reference, measured_angle);
    }

    void set_reference(const double reference_){
        reference = reference_;
    }

    void set_kp(const double kp_){
        kp = kp_;
    }
    void set_ki(const double ki_){
        ki = ki_;
    }
    void set_kd(const double kd_){
        kd = kd_;
    }

    void set_pid_constants(const double kp_, const double ki_, const double kd_){
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }
    double get_kp(){
        return kp;
    }
    double get_ki(){
        return ki;
    }
    double get_kd(){
        return kd;
    }
    double get_reference(){
        return reference;
    }
    
private:
double reference{0}, setpoint{0};
double kp{0}, ki{0}, kd{0}, i{0}, last_error{0}, error{0};
std::function<double(double, double)> pid_update;
std::chrono::steady_clock::time_point last_time;
};