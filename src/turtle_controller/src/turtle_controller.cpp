#include <chrono>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
// #include "turtlesim/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtle_controller/FSM.hpp"
#include "turtle_controller/risk_model.hpp"

constexpr double X_SPEED = 0.30;
constexpr double X_SPEED_TRANSITION = 0.22;
constexpr double X_SPEED_ESCAPE = 0.16;

constexpr double MAX_ANGULAR = 2;
using namespace std::chrono_literals;
using std::placeholders::_1;

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
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TurtleController::pose_callback, this, _1));
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
            "State: %d, right_risk: %.2f, left_risk:%.2f, front_risk:%.2f,  front_wide_risk:%.2f,  back_risk:%.2f, global_risk:%.2f",
            static_cast<int>(state_), right_risk_, left_risk_, front_risk_, front_wide_risk_, back_risk_, global_risk_);

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

        message.linear.x = X_SPEED_ESCAPE * (1.0 - global_risk_);
        message.angular.z = turn * escape_k_mult;

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

        message.linear.x = X_SPEED_TRANSITION * (1.0 - global_risk_); // più veloce ---- più rischio
        message.angular.z = turn * (1.0 - global_risk_);

        previous_turn_ = turn;
    }

    void handle_explore(geometry_msgs::msg::Twist &message)
    {
        auto now = this->now();

        if ((now - explore_time_direction_change_).seconds() > min_duration_explore_)
        {
            explore_time_direction_change_ = now;
            explore_turn_ = dist(gen) * 0.6;
        }

        double turn =
            explore_k_weight_ * explore_turn_ +
            0.2 * previous_turn_;

        message.linear.x = X_SPEED * (1.0 - global_risk_); // più veloce --- più rischio;
        message.angular.z = turn * (1.0 - global_risk_);

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

    // declaration of the publisher and subscription as private members of the class TurtleController
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Time explore_time_direction_change_; // time when turtle changes direction while exploring
    State state_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    // dynamic memory for istance FSM and RiskModel
    std::unique_ptr<FSM> fsm_;
    std::unique_ptr<RiskModel> risk_model_;

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
