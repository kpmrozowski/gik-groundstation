#include <memory>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/builtin_uint64.h>

// #include <fmt/format.h>

class VolleyballNode {
    boost::function<void (const std_msgs::UInt64::ConstPtr&)> callback_jetson_;
    boost::function<void (const std_msgs::UInt64::ConstPtr&)> callback_ground_;
    ros::Subscriber sub_to_jetson_;
    ros::Subscriber sub_to_ground_;
    ros::Publisher pub_to_jetson_;
    ros::Publisher pub_to_ground_;

public:
    VolleyballNode(ros::NodeHandle nh) {
        callback_jetson_ = boost::function<void (const std_msgs::UInt64::ConstPtr&)>(
            [this] (const std_msgs::UInt64::ConstPtr& msg_received) -> void {
                ROS_INFO("I heard: [%zu] from jetson", msg_received->data);
                std_msgs::UInt64 msg_to_send;
                msg_to_send.data = msg_received->data + 1;
                // usleep(5000);
                pub_to_jetson_.publish(msg_to_send);
            }
        );
        callback_ground_ = boost::function<void (const std_msgs::UInt64::ConstPtr&)>(
            [this] (const std_msgs::UInt64::ConstPtr& msg_received) -> void {
                ROS_INFO("I heard: [%zu] from ground", msg_received->data);
                std_msgs::UInt64 msg_to_send;
                msg_to_send.data = msg_received->data + 1;
                // usleep(5000);
                pub_to_ground_.publish(msg_to_send);
            }
        );
        pub_to_jetson_ = nh.advertise<std_msgs::UInt64>("rpi_to_jetson", 1000);
        pub_to_ground_ = nh.advertise<std_msgs::UInt64>("rpi_to_ground", 1000);
        sub_to_jetson_ = nh.subscribe("jetson_to_rpi", 1000, callback_jetson_);
        sub_to_ground_ = nh.subscribe("ground_to_rpi", 1000, callback_ground_);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "player_rpi");
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");

    VolleyballNode valley_ball_node(nh);

    ros::Rate rate(24.);

    while(ros::ok())
    {
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    ros::spinOnce();
        rate.sleep();
    }
    printf("IT'S MILLER TIME! :D");
    return 0;
}
