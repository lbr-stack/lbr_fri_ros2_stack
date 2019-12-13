#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <fri/friLBRState.h>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {

    public:
        Publisher() 
            : Node("fri_test_publisher"),
              _freqHz(0.25),
              _amplRad(0.04),
              _filterCoeff(0.99),
              _offset(0.0),
              _phi(0.0),
              _stepWidth(0.0) {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lbr_joint_angle_in", 10);
            timer_ = this->create_wall_timer(5ms, std::bind(&Publisher::timer_callback, this));   

            // copy KUKA's example
            _stepWidth = 2 * M_PI * _freqHz * 5; // robotState().getSampleTime();
        }

    private:
        void timer_callback() {
            RCLCPP_INFO(this->get_logger(), "%f", _offset);

            // calculate new offset
            double newOffset = _amplRad * sin(_phi);
            _offset = _offset * _filterCoeff + newOffset * (1.0 - _filterCoeff);
            _phi += _stepWidth;
            if (_phi >= 2 * M_PI) _phi -= 2 * M_PI;     

            std::vector<double> out{_offset, 0., 0., 0., 0., 0., 0.};

            auto msg = std_msgs::msg::Float64MultiArray();
            msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            msg.layout.dim[0].size = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "i";

            msg.data.clear();
            msg.data.insert(msg.data.end(), out.begin(), out.end());

            pub_->publish(msg);
        };

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

        double _freqHz;         //!< sine frequency (Hertz)
        double _amplRad;        //!< sine amplitude (radians)
        double _filterCoeff;    //!< filter coefficient
        double _offset;         //!< offset for current interpolation step
        double _phi;            //!< phase of sine wave
        double _stepWidth;      //!< stepwidth for sine 
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}