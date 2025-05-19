#include "safety_system_node.hpp"

SafetyNode::SafetyNode () : Node("SafetySystem") {

    //ttc brekaing and timeing parameters (tune to get best results)
    //Throtel values must add up to 1 
    pram.TTC_stage1 = 1.24;
    pram.TTC_stage2 = 0.64;
    pram.TTC_stage3 = 0.32;
    pram.TTC1_Throtel = 0.4;
    pram.TTC2_Throtel = 0.2;
    pram.TTC3_Throtel = 0.4;

    //vehicle physical parameters
    pram.WHEEL_RADIUS = 0.0381; // in meters

    //nose and threshold adustments
    pram.alarm_threshold = 3; // the number consecutive of alerts before ttc trigers
    pram.velocity_threshold = 0.01; // adjust for noise in velocity mesurmnet

    //important note all of these guys must be less than or = to 1
    pram.encoder_bias = 0.5; // bias for the encoders since one can be more reliable than ther other
    pram.alpha = 0.65; // the sensor bias in the complementary filter
    pram.lp_factor = 0.75; // lowe pass filter bias (higher = better response but more noise)

    //initalizing some starting parameters
    imu_velocity = 0; // meters/second (always 0 at the start for both)
    encoder_last_rad = 0;
    encoder_velocity = 0; 
    last_accl = 0;

    //define the subs
    laider_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/autodrive/f1tenth_1/lidar", 10, std::bind(&SafetyNode::laiderCallBack, this, std::placeholders::_1));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "/autodrive/f1tenth_1/imu",10, std::bind(&SafetyNode::imuCallBack,this,std::placeholders::_1));

    left_encoder_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/autodrive/f1tenth_1/left_encoder",10,[this](const sensor_msgs::msg::JointState::SharedPtr msg){left_encoder_data = msg;});

    right_encoder_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/autodrive/f1tenth_1/right_encoder",10,[this](const sensor_msgs::msg::JointState::SharedPtr msg){right_encoder_data = msg;});

    //define the pub
    vel_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command",10);

    //define the fist time instance
    last_imu_time = this->now();
    last_encoder_time = this->now();

    RCLCPP_INFO(this->get_logger(),"initalized saftey constructor");
   
} 

//takes current imu data and produeces a linear velocity reading
void SafetyNode::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr data) {

    //figure out the time delta between this and last call back
    rclcpp::Time current = data->header.stamp;
    double delta = std::abs(current.seconds() - last_imu_time.seconds());
    last_imu_time = current; 

    //low pass filter the accleration (to reduce noise and drift)
    double filtered_acel = pram.lp_factor * data->linear_acceleration.x + (1 - pram.lp_factor) * last_accl;
    last_accl = filtered_acel;

    //accumilate the velocity
    imu_velocity += filtered_acel * delta;

    //log the current imu velocity
    //RCLCPP_INFO(this->get_logger(),"the current imu velocity is %f",imu_velocity);

}

//uses the current encoder values and last to find velocity
void SafetyNode::findEncoderVelocity() {
    
    //figure out the current delta (might be better to take the avrage of the 2 encoder stamps at th time)
    rclcpp::Time current = this->now();
    double delta = std::abs(current.seconds() - last_encoder_time.seconds());
    last_encoder_time = current;

    if (delta <= 0)
        return;

    //get get the current encoder postion
    double current_encoder_rad = right_encoder_data->position[0] * pram.encoder_bias + left_encoder_data->position[0] * (1 - pram.encoder_bias);
    encoder_velocity = ((current_encoder_rad - encoder_last_rad) * pram.WHEEL_RADIUS) / delta;

    //store the curret value as last for future
    encoder_last_rad = current_encoder_rad;

    //log the current encoder velocity
    //RCLCPP_INFO(this->get_logger(),"the current encoder velocity is %f",encoder_velocity);

}

void SafetyNode::laiderCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laider_msg) {
    
    float throtel_input = 0;
    int tringer_counter = 0;
    bool sending_break = false;
    std_msgs::msg::Float32 msg;

    //find the encoder velocity at the given time
    SafetyNode::findEncoderVelocity(); 

    //if the encoder has a velocity of 0 then so should the imu in theory
    if (std::abs(encoder_velocity) <= pram.velocity_threshold) {
        imu_velocity = 0;
    }   

    //do a complementary filter, basicaly a weighted mean, 
    //bias towards imu as wheels could making encoder less reliable slip
    double velocity_estimte = pram.alpha * imu_velocity + (1 - pram.alpha) * encoder_velocity;
    
    //log the current velocity
    RCLCPP_INFO(this->get_logger(),"the current estimated velocity is %f",velocity_estimte);

    //skip if the vehicle velocity is 0, as ttc if than infinity (for now return a 0 cuase there is controls to correct it)
    if (velocity_estimte < pram.velocity_threshold) {
        msg.data = throtel_input;
        vel_pub->publish(msg);
        return;
    }

    for (size_t i = 0 ; i < laider_msg->ranges.size(); i++) {

        //no need to check values outside range
        if (laider_msg->ranges[i] > laider_msg->range_max || laider_msg->ranges[i] < laider_msg->range_min) {
            continue;
        }

        float angle = laider_msg->angle_min + i * laider_msg->angle_increment;
        float range_dt = velocity_estimte * std::cos(angle);

        //if rate is positive or 0 than the ttc is increase, so safe, or is infinity so also safe
        if (range_dt >= 0) {
            continue;
        }

        //calculate and long ttc
        float ttc = laider_msg->ranges[i] / -range_dt;
        RCLCPP_INFO(this->get_logger(),"the ttc = %4f",ttc);

        /*insted of a niave approch and relying on one beam, 
        its better to see if 3 ttcs are triggered below the thereshorld before 
        sending the break signal thats why we get less false positives*/

        //check if the ttc is trigered of discard it as false alarm and reset
        if (ttc < pram.TTC_stage1) {
            //RCLCPP_INFO(this->get_logger(),"stage 1 triggered");
            throtel_input -= pram.TTC1_Throtel;
            if (ttc < pram.TTC_stage2) {
                //RCLCPP_INFO(this->get_logger(),"stage 2 triggered");
                throtel_input -= pram.TTC2_Throtel;
                if (ttc < pram.TTC_stage3) {
                    //RCLCPP_INFO(this->get_logger(),"stage 3 triggered");
                    throtel_input -= pram.TTC3_Throtel;
                }
            }
            tringer_counter++;
        } else {
            //RCLCPP_INFO(this->get_logger(),"false alarm");
            tringer_counter = 0;
            throtel_input = 0.0;
        }

        if (tringer_counter >= pram.alarm_threshold) {
            sending_break = true;
            break;
        }

    }

    if (sending_break) {

        if (throtel_input > 0)
            throtel_input = 0;

        else if (throtel_input < -1.0)
            throtel_input = -1.0;
        
        RCLCPP_INFO(this->get_logger(),"sending throtel %f",throtel_input);

        msg.data = throtel_input;
        vel_pub->publish(msg);

    } 

}

int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}

