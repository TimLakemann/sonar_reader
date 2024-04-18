#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/asio.hpp>
#include <std_msgs/Int32.h>
#include <mrs_lib/param_loader.h>

namespace sonar{
    class SonarReader: public nodelet::Nodelet {
    public:
        SonarReader(): serial_port_(io_){}

        virtual void onInit() {
            nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
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
                boost::asio::read_until(serial_port_, buffer, "\r");
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
                ROS_INFO("Distance: %d cm", distance);
                std_msgs::Int32 msg;
                msg.data = distance;
                pub_.publish(msg);
            }
        }

        boost::asio::io_service io_;
        boost::asio::serial_port serial_port_;
        ros::Publisher pub_;
        ros::NodeHandle nh_;
    };

}
PLUGINLIB_EXPORT_CLASS(sonar::SonarReader, nodelet::Nodelet)
