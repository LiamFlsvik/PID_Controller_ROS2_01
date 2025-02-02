
#include "pidController.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

class PIDNode : public rclcpp::Node {

public:
    PIDNode(): Node("pid_controller_node"),
        pidController_(0.001, 0.000008, 0.001), reference_(10.0), measured_angle(0.0) { 

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("Voltage", 10);
        
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>("Angle", 10, std::bind(&PIDNode::processVariableCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PIDNode::computeAndPublish, this));

        declare_parameter("kp", pidController_.get_kp());
        declare_parameter("ki", pidController_.get_ki());
        declare_parameter("kd", pidController_.get_kd());
        declare_parameter("reference", pidController_.get_reference());
        
        
        
        parameter_cb_handle = this->add_on_set_parameters_callback(
        std::bind(&PIDNode::parameterCallback, this, std::placeholders::_1));
        
    }


private:

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> & params){
        // Fail
        rcl_interfaces::msg::SetParametersResult result;
        
    
        kp = get_parameter("kp").as_double();
        ki = get_parameter("ki").as_double();
        kd = get_parameter("kd").as_double();
        reference_ = this->get_parameter("reference").as_double();

        if(params.size() == 0){
            result.successful = false;
            result.reason = "No parameters passed";
            return result;
        }

        for(auto p: params) {
            if(p.get_name()=="reference"){reference_ = p.as_double();}
            if(p.get_name()=="kp"){kp = p.as_double();}
            if(p.get_name()=="ki"){ki = p.as_double();}
            if(p.get_name()=="kd"){kd = p.as_double();}
            
        }
        

        if(kp < 0 || ki < 0 || kd < 0){
            result.successful = false;
            result.reason = "Parameter < 0";
            return result;
        }

        this->pidController_.set_pid_constants(kp, ki, kd);
        this->pidController_.set_reference(reference_);
    
        //RCLCPP_INFO(this->get_logger(), "PID constants: kp = %f, ki = %f, kd = %f, reference =%f ", kp, ki, kd, reference_);
    
        result.successful = true;
        result.reason = "Parameters successfully set";
        return result;
    }

    void processVariableCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        measured_angle = msg->data;
        pidController_.set_reference(reference_);
    }

    void computeAndPublish() {
        double control_signal = pidController_.update(reference_, measured_angle);
        auto message = std_msgs::msg::Float64();
        message.data = control_signal;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Control Signal: %.2f, Measured Angle: %.2f, Reference: %.2f", control_signal, measured_angle, pidController_.get_reference());
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle;

    pidController pidController_;
    double reference_;
    double measured_angle;
    double kp, ki, kd;
    
};