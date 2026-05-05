#include <chrono>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <algorithm>
// #include "turtlesim/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtle_controller/FSM.hpp"
#include "turtle_controller/risk_model.hpp"
#include "nav_msgs/msg/odometry.hpp"



constexpr double X_SPEED = 0.30;
constexpr double X_SPEED_TRANSITION = 0.22;
constexpr double X_SPEED_ESCAPE = 0.16;

constexpr double MAX_ANGULAR = 2;
using namespace std::chrono_literals;
using std::placeholders::_1;



/*


francesco@francesco-HP-Laptop-15s-fq5xxx:~/Desktop/ros2_intelligent_navigation_ws$ ros2 interface show nav_msgs/msg/Odometry
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose
	Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	float64[36] covariance

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
	Twist twist
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	float64[36] covariance




*/

/*
francescotief@francesco-tief:~$ ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
francescotief@francesco-tief:~$
*/




class VelocityControllerPI
{
public:
    VelocityControllerPI(double kp = 1.0,
                         double ki = 0.3,
                         double ts = 0.05)
        : Kp_(kp), Ki_(ki), Ts_(ts)
    {}

    void setReference(double v_ref)
    {
        v_ref_ = v_ref;
    }

    void setMeasurement(double v_meas)
    {
        v_meas_ = v_meas;
    }

    void update()
    {
        double error = v_ref_ - v_meas_;

        // integratore discreto
        integral_ += error * Ts_;

        // anti-windup semplice
        integral_ = std::clamp(integral_, -2.0, 2.0);

        // PI
        u_ = Kp_ * error + Ki_ * integral_;
    }

    double getCommand() const
    {
        return u_;
    }

    void reset()
    {
        integral_ = 0.0;
        u_ = 0.0;
        v_ref_ = 0.0;
        v_meas_ = 0.0;
    }

private:
    double Kp_;
    double Ki_;
    double Ts_;

    double v_ref_ = 0.0;
    double v_meas_ = 0.0;
    double u_ = 0.0;

    double integral_ = 0.0;
};

class TurtleController : public rclcpp ::Node
{
public:
    // node constructor
    TurtleController() : Node("turtle_controller"), gen(rd()), dist(-1.0, 1.0)
    {
        // in the section of the constructor I create a publisher (on topic /turtle1/cmd_vel) and a subscription (on topic /turtle1/pose)
        this->declare_parameter<double>("min_duration_explore", 1.5);
        this->declare_parameter<double>("explore_k_weight", 0.8);
        this->declare_parameter<double>("transition_k_random", 0.10);
        this->declare_parameter<double>("escape_k_mult", 1.5);
        min_duration_explore_ = this->get_parameter("min_duration_explore").as_double();
        explore_k_weight_ = this->get_parameter("explore_k_weight").as_double();
        escape_k_mult = this->get_parameter("escape_k_mult").as_double();
        transition_k_random_ = this->get_parameter("transition_k_random").as_double();
        fsm_ = std::make_unique<FSM>();
        risk_model_ = std::make_unique<RiskModel>();
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_ = fsm_->getCurrentState();
        explore_time_direction_change_ = this->now();
        explore_turn_ = dist(gen);
        pi_linear_ = VelocityControllerPI(1.2, 0.6, 0.05);
        pi_angular_ = VelocityControllerPI(1.0, 0.3, 0.05);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TurtleController::pose_callback, this, _1));
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TurtleController::odom_callback, this, _1));

    }

private:
    // definizione della callback (viene chiamata ogni volta che il nodo riceve un messaggio di tipo sensor_msgs::msg::laser_scan)
    void pose_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // qui pubblichiamo un messaggio di tipo geometry_msgs::msg::Twist per controllare la tartaruga
        // here we create a message of type geometry_msgs::msg::Twist to control the turtle
        // auto message = geometry_msgs::msg::Twist();

        last_scan_ = msg;
        if (!last_scan_)
            return;

        auto message = geometry_msgs::msg::Twist();
        risk_model_->updateScan(last_scan_);
        risk_model_->update();
        global_risk_ = risk_model_->getGlobalRisk();
        front_risk_ = risk_model_->getFrontRisk();
        front_wide_risk_ = risk_model_->getFrontWideRisk();
        left_risk_ = risk_model_->getLeftRisk();
        right_risk_ = risk_model_->getRightRisk();
        back_risk_ = risk_model_->getBackRisk();
        state_ = fsm_->update(risk_model_->getGlobalRisk());

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            300,
            "State: %d, right_risk: %.2f, left_risk:%.2f, front_risk:%.2f,  front_wide_risk:%.2f,  back_risk:%.2f, global_risk:%.2f, v=%.2f w=%.2f",
            static_cast<int>(state_), right_risk_, left_risk_, front_risk_, front_wide_risk_, back_risk_, global_risk_,current_linear_velocity_, current_angular_velocity_);

        switch (state_)
        {
        case State::ESCAPE:
            handle_escape(message);
            break;
        case State::TRANSITION:
            handle_transition(message);
            break;
        default:
            handle_explore(message);
            break;
        }

        // publish the message to control the turtle on the topic /turtle1/cmd_vel
        publisher_->publish(message);
    }


    // INIZIO PORTING HANDLE IN NUOVA CLASSE
    void handle_escape(geometry_msgs::msg::Twist &message)
    {
        double dir = chooseEscapeDirection();        // lato suggerito
        double repulsion = right_risk_ - left_risk_; // forza laterale

        double turn =
            0.7 * dir +      // direzione scelta
            0.3 * repulsion; // intensità reale del rischio

        double v_ref = X_SPEED_ESCAPE * (1.0 - global_risk_);
        double w_ref = turn * (1.0 - global_risk_);

        
        pi_linear_.setReference(v_ref);
        pi_angular_.setReference(w_ref);

        // 2. measurement (da odom)
        pi_linear_.setMeasurement(current_linear_velocity_);
        pi_angular_.setMeasurement(current_angular_velocity_);
        pi_angular_.update();
        pi_linear_.update();
        double u_linear = pi_linear_.getCommand();
        double w_angular = pi_angular_.getCommand();
        v_ref_smooth_ = 0.2 * v_ref_smooth_ + 0.8 * u_linear;
        w_ref_smooth_ = 0.25 * w_ref_smooth_ + 0.75 * w_angular;
        message.linear.x = v_ref_smooth_;
        message.angular.z = w_ref_smooth_;

        //message.linear.x  = pi_linear_.update(v_ref_smooth_, current_linear_velocity_,now);
        
     


        //message.linear.x = std::clamp(message.linear.x, -X_SPEED_ESCAPE, X_SPEED_ESCAPE);
        //message.angular.z = std::clamp(message.angular.z, -MAX_ANGULAR, MAX_ANGULAR);

        /*
        message.linear.x = X_SPEED_ESCAPE * (1.0 - global_risk_);
        message.angular.z = turn * escape_k_mult;
        */
        previous_turn_ = turn;
    }

    void handle_transition(geometry_msgs::msg::Twist &message)
    {
        double dir = chooseEscapeDirection();        // lato suggerito
        double repulsion = right_risk_ - left_risk_; // forza reale
        double explore_bias = dist(gen);             // componente esplorativa

        // TRANSITION più rischiosa:
        // - meno repulsione
        // - più avanzamento
        // - meno rotazione
        // - memoria più forte

        double turn =
            0.35 * dir +                         // direzione suggerita
            0.20 * repulsion +                   // repulsione moderata
            0.35 * previous_turn_ +              // memoria forte
            transition_k_random_ * explore_bias; // esplorazione leggera


        double v_ref = X_SPEED_TRANSITION * (1.0 - global_risk_);
        double w_ref = turn * (1.0 - global_risk_);

        // filtro passa-basso sul riferimento
   
        pi_linear_.setReference(v_ref);
        pi_angular_.setReference(w_ref);

        // 2. measurement (da odom)
        pi_linear_.setMeasurement(current_linear_velocity_);
        pi_angular_.setMeasurement(current_angular_velocity_);
        pi_angular_.update();
        pi_linear_.update();
        double u_linear = pi_linear_.getCommand();
        double w_angular = pi_angular_.getCommand();
        v_ref_smooth_ = 0.2 * v_ref_smooth_ + 0.8 * u_linear;
        w_ref_smooth_ = 0.25 * w_ref_smooth_ + 0.75 * w_angular;
        message.linear.x = v_ref_smooth_;
        message.angular.z = w_ref_smooth_;

        /*
        message.linear.x = X_SPEED_TRANSITION * (1.0 - global_risk_); // più veloce ---- più rischio
        message.angular.z = turn * (1.0 - global_risk_);
        */
        previous_turn_ = turn;
    }

    void handle_explore(geometry_msgs::msg::Twist &message)
    {
        if ((this->now() - explore_time_direction_change_).seconds() > min_duration_explore_)
        {
            explore_time_direction_change_ = this->now();
            explore_turn_ = dist(gen) * 0.6;
        }

        double turn =
            explore_k_weight_ * explore_turn_ +
            0.2 * previous_turn_;

        double v_ref = X_SPEED * (1.0 - global_risk_);
        double w_ref = turn * (1.0 - global_risk_);

        // filtro passa-basso sul riferimento
   
        pi_linear_.setReference(v_ref);
        pi_angular_.setReference(w_ref);

        // 2. measurement (da odom)
        pi_linear_.setMeasurement(current_linear_velocity_);
        pi_angular_.setMeasurement(current_angular_velocity_);
        pi_angular_.update();
        pi_linear_.update();
        double u_linear = pi_linear_.getCommand();
        double w_angular = pi_angular_.getCommand();
        v_ref_smooth_ = 0.2 * v_ref_smooth_ + 0.8 * u_linear;
        w_ref_smooth_ = 0.25 * w_ref_smooth_ + 0.75 * w_angular;
        message.linear.x = v_ref_smooth_;
        message.angular.z = w_ref_smooth_;

        //message.linear.x = std::clamp(message.linear.x, -X_SPEED_ESCAPE, X_SPEED_ESCAPE);
        //message.angular.z = std::clamp(message.angular.z, -MAX_ANGULAR, MAX_ANGULAR);

            /*    

        message.linear.x = X_SPEED * (1.0 - global_risk_); // più veloce --- più rischio;
        message.angular.z = turn * (1.0 - global_risk_);
        */
        previous_turn_ = turn;
    }

    // FINE PORTING HANDLE

    double chooseEscapeDirection()
    {

        if (front_risk_ > 0.25)
        {
            if (left_risk_ < right_risk_)
                return 1.0; // gira a sinistra
            else
                return -1.0; // gira a destra
        }

        // 2. Se il fronte è moderato -- usa front_wide per capire la direzione
        if (front_wide_risk_ > 0.20)
        {
            if (left_risk_ < right_risk_)
                return 0.7; // sinistra, ma più dolce
            else
                return -0.7; // destra, ma più dolce
        }

        // 3. Se il fronte è libero -- scegli il lato meno rischioso
        if (left_risk_ < right_risk_)
            return 0.5;
        else if (right_risk_ < left_risk_)
            return -0.5;

        // 4. Se sono uguali -- mantieni la direzione precedente
        return previous_turn_ >= 0 ? 0.5 : -0.5;
    }   

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_linear_velocity_ = msg->twist.twist.linear.x;
        current_angular_velocity_ = msg->twist.twist.angular.z;
    }


 

    double current_linear_velocity_ = 0.0;
    double current_angular_velocity_ = 0.0;

    // declaration of the publisher and subscription as private members of the class TurtleController
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Time explore_time_direction_change_; // time when turtle changes direction while exploring
    State state_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    // dynamic memory for istance FSM and RiskModel
    std::unique_ptr<FSM> fsm_;
    std::unique_ptr<RiskModel> risk_model_;

    VelocityControllerPI pi_linear_;
    VelocityControllerPI pi_angular_;

    double v_ref_smooth_ = 0.0;
    double w_ref_smooth_ = 0.0;

    // utilty to generate randomic number
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<double> dist;

    // ROS 2 Parameters
    double explore_turn_; // variable to store the random turn value
    double min_duration_explore_;
    double explore_k_weight_;
    double escape_k_mult;
    double previous_turn_;
    double transition_k_random_;

    // risks
    double global_risk_;
    double left_risk_;
    double right_risk_;
    double front_risk_;
    double front_wide_risk_;
    double back_risk_;
};

int main(int argc, char *argv[])
{
    // initialize the ROS 2 system, create an instance of the TurtleController node, and spin it to process callbacks
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}
