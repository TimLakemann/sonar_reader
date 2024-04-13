#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/asio.hpp>
#include <mrs_lib/param_loader.h>

class SerialReaderNode {
public:
    SerialReaderNode() : nh_("~"), serial_port_(io_) {
        // Get serial port parameters from ROS parameter server
        std::string _portname_;
        int _baud_rate_;
        std::string _uav_name_;
 
        mrs_lib::ParamLoader param_loader(nh_, "SerialReaderNode");
        param_loader.loadParam("uav_name", _uav_name_, std::string("uav1"));
        param_loader.loadParam("portname", _portname_, std::string("/dev/ttyACM0"));
        param_loader.loadParam("baud_rate", _baud_rate_, int(9600));

        serial_port_.open(_portname_);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(_baud_rate_));
        // Create publisher
        pub_ = nh_.advertise<std_msgs::Int32>("sonar_distance", 1);
        // Start reading from serial port
        readSerialData();
    }
private:
    void readSerialData() {
        while (ros::ok()) {
            boost::asio::streambuf buffer;
            boost::asio::read_until(serial_port_, buffer, "\n");
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);
            // Publish the data
            
            int distance;
            if(line[0] == 'R' && line.size() > 1){
                line.erase(0,1);
                distance  = stoi(line);
            }else{
                distance = -1;
            }
            
            std_msgs::Int32 msg;
            msg.data = distance;

            pub_.publish(msg);
        }
    }
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_reader_node");
    SerialReaderNode sonar_reader_node;
    
    ros::spin();
    return 0;
}