#include <ros/ros.h>
#include <std_msgs/String.h>

class ListenerArob{

    private:

    std::map<std::string, ros::Subscriber> suscribers; 
    std::map<std::string, ros::Publisher> publishers; 

    void requestCallback(const std_msgs::String::ConstPtr& msg){
        
        ROS_INFO_STREAM("Received: " << msg->data.c_str());

        std_msgs::String response;
        response.data = "Hello, I am a student";
        publishers["responseArob"].publish(response);

        ROS_INFO_STREAM("Published: " << response.data.c_str());
    }

    public:

    int main(int argc, char** argv){

        ros::init(argc, argv, "listenerArob");

        ros::NodeHandle nodeHandle;

        suscribers["requestArob"] = nodeHandle.subscribe("requestArob", 1000, &ListenerArob::requestCallback, this);
        publishers["responseArob"] = nodeHandle.advertise<std_msgs::String>("responseArob", 1000);

        ros::spin();

        return 0;
    }

};

int main(int argc, char** argv)
{
    ListenerArob listener;
    return listener.main(argc, argv);
}

