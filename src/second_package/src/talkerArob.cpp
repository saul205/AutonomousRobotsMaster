#include <ros/ros.h>
#include <std_msgs/String.h>

class TalkerArob{

    private:

    std::map<std::string, ros::Subscriber> suscribers; 
    std::map<std::string, ros::Publisher> publishers; 

    void sendMsg(std_msgs::String msg, ros::Publisher pub){
        pub.publish(msg);
        ROS_INFO_STREAM("Published: " << msg.data.c_str());
    }

    void responseCallback(const std_msgs::String::ConstPtr& msg){
        ROS_INFO_STREAM("Received: " << msg->data.c_str());

        std_msgs::String response;
        response.data = "Autonomous Robot Course";
        sendMsg(response, publishers["requestArob"]);
    }

    public:

    int main(int argc, char** argv){

        ros::init(argc, argv, "talkerArob");

        ros::NodeHandle nodeHandle;

        suscribers["responseArob"] = nodeHandle.subscribe("responseArob", 1000, &TalkerArob::responseCallback, this);
        publishers["requestArob"] = nodeHandle.advertise<std_msgs::String>("requestArob", 1000);

        std_msgs::String msg;
        msg.data = "Autonomous Robot Course";

        sendMsg(msg, publishers["requestArob"]);

        ros::spin();

        return 0;
    }

};

int main(int argc, char** argv)
{
    TalkerArob talker;
    return talker.main(argc, argv);
}