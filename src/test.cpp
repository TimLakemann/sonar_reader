#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/asio.hpp>

class SerialReaderNode {
public:
    SerialReaderNode() : nh_("~"), serial_port_(io_) {
        // Get serial port parameters from ROS parameter server
        std::string serial_port_name;
        int baud_rate;
        nh_.param<std::string>("serial_port", serial_port_name, "/dev/ttyACM0");
        nh_.param<int>("baud_rate", baud_rate, 9600);

        // Open serial port
        serial_port_.open(serial_port_name);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

        // Create publisher
        pub_ = nh_.advertise<std_msgs::String>("serial_data", 10);

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
            std_msgs::String msg;
            msg.data = line;
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
    SerialReaderNode serial_reader_node;
    ros::spin();
    return 0;
}
