#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <signal.h>
#include "scan_data_receiver.h"
#include "visioscan_rd_driver.h"

namespace bea_power{
bool to_exit = false;

class BEANode : public rclcpp::Node
{
public:
    BEANode() : Node("visioscan_node")
    {

    }

    int work_loop()
    {
        init_param();

        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_id, rclcpp::QoS(rclcpp::KeepLast(100)));

        driver = new VISIOSCANDriver();

        std::cout << "Connecting to scanner at " << laser_ip << " ... " << std::endl;
        while(!driver->connect(laser_ip, laser_port))
        {
            std::cout << "Connecting to scanner at " << laser_ip << ":" << laser_port << " failed! Retrying..."
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        std::cout << "@BEANode::connect OK" << std::endl;

        auto params = driver->GetParameters();
        float angleMin, angleMax, deltaAngle;
        int skipSpots = params.skipSpots;
        float angleResol = params.angularResolution;
        if(params.scanDataDirection == 0)
        {
#if MDI_PACKET_HEADER_SYNC_0 == 0xBE && MDI_PACKET_HEADER_SYNC_1 == 0xA0
            angleMin = (params.stopAngle - 90.0) * TO_RADIAN;
            angleMax = (params.startAngle - 90.0) * TO_RADIAN;
            deltaAngle = -angleResol * (skipSpots + 1) * TO_RADIAN;
#elif MDI_PACKET_HEADER_SYNC_0 == 0x4C && MDI_PACKET_HEADER_SYNC_1 == 0x45
            angleMin = (-params.startAngle) * TO_RADIAN;
            angleMax = (-params.stopAngle) * TO_RADIAN;
            deltaAngle = -angleResol * (skipSpots + 1) * TO_RADIAN;
#endif
        }
        else
        {
#if MDI_PACKET_HEADER_SYNC_0 == 0xBE && MDI_PACKET_HEADER_SYNC_1 == 0xA0
            angleMin = (params.startAngle - 90.0) * TO_RADIAN;
            angleMax = (params.stopAngle - 90.0) * TO_RADIAN;
            deltaAngle = angleResol * (skipSpots + 1) * TO_RADIAN;
#elif MDI_PACKET_HEADER_SYNC_0 == 0x4C && MDI_PACKET_HEADER_SYNC_1 == 0x45
            angleMin = (-params.stopAngle) * TO_RADIAN;
            angleMax = (-params.startAngle) * TO_RADIAN;
            deltaAngle = angleResol * (skipSpots + 1) * TO_RADIAN;
#endif
        }

        if(laser_direction)
        {
            angleMin = -angleMin;
            angleMax = -angleMax;
            deltaAngle = -deltaAngle;
        }
        
        double scan_duration = params.scan_time;
        int packet_type = params.lidarDataPacketType;
        int protocol = params.protocolType;

        if(protocol == 0)
        {
            if(driver->startCapturingUDP())
            {
                std::cout << "OK" << std::endl;
            }
            else
            {
                std::cout << "FAILED" << std::endl;
                return -1;
            }
        }
        else if(protocol == 1)
        {
            if(driver->startCapturingTCP())
            {
                std::cout << "OK" << std::endl;
            }
            else
            {
                std::cout << "FAILED" << std::endl;
                return -1;
            }
        }

        rclcpp::Time start_scan_time;
        while(rclcpp::ok() && !to_exit)
        {
            int count = driver->getFullScansAvailable();
            for(int i = 0; i < count; i++)
            {
                auto scandata = driver->getFullScan();

                start_scan_time = this->now();
                int scansize = scandata.distance_data.size();
                if(scansize)
                {
                    this->publish_scan(scan_pub, 
                                    scandata, scansize,
                                    start_scan_time, scan_duration,
                                    angleMin, angleMax, deltaAngle,
                                    packet_type, frame_id);
                }
            }
            //std::cout << "Received " << count << " from scanner" << std::endl;
            usleep(20 * 1000);
            //RCLCPP_INFO(this->get_logger(), "The sensor ip and port are : [%s, %d]", laser_ip.c_str(), laser_port);
            rclcpp::spin_some(shared_from_this());
        }

        if(driver) {delete driver; driver = nullptr;}
        return 0;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

    std::string frame_id;
    std::string laser_ip;
    int laser_port;
    int scan_frequency;
    int samples_per_scan;
    int laser_direction;
    std::string topic_id;
    VISIOSCANDriver* driver;

    void init_param()
    {
        this->declare_parameter<std::string>("frame_id", "laser");
        this->declare_parameter<std::string>("topic_id", "scan");
        this->declare_parameter<std::string>("laser_ip", "192.168.1.2");
        this->declare_parameter<int>("laser_port", 3050);
        this->declare_parameter<int>("scan_frequency", 80);
        this->declare_parameter<int>("samples_per_scan", 1377);
        this->declare_parameter<int>("laser_direction", 0);

        this->get_parameter_or<std::string>("frame_id", frame_id, "laser");
        this->get_parameter_or<std::string>("topic_id", topic_id, "scan");
        this->get_parameter_or<std::string>("laser_ip", laser_ip, "192.168.1.2");
        this->get_parameter_or<int>("laser_port", laser_port, 3050);
        this->get_parameter_or<int>("scan_frequency", scan_frequency, 80);
        this->get_parameter_or<int>("samples_per_scan", samples_per_scan, 1377);
        this->get_parameter_or<int>("laser_direction", laser_direction, 0);
    }

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                      ScanData scan, int count, 
                      rclcpp::Time start, double scan_time, 
                      float angle_min, float angle_max, float angle_delta,
                      int packet_type, std::string frame_id)
    {
        //static int scan_count = 0;
        auto scanmsg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scanmsg->header.stamp = start;
        scanmsg->header.frame_id = frame_id;
        //scan_count++;

        scanmsg->angle_min = angle_min;
        scanmsg->angle_max = angle_max;
        scanmsg->angle_increment = angle_delta;

        scanmsg->scan_time = scan_time;
        scanmsg->time_increment = scan_time / (double)(count - 1);
        scanmsg->range_min = 0.08;
        scanmsg->range_max = ROS_RANGE_MAX;

        if(packet_type == 1)
        {
            scanmsg->ranges.resize(count);
            scanmsg->intensities.resize(count);
            for(int i = 0; i < count; i++)
            {
                scanmsg->ranges[i] = float(scan.distance_data.at(i)) / 1000.0f;
                scanmsg->intensities[i] = float(scan.amplitude_data.at(i));
            }
        }
        else
        {
            scanmsg->ranges.resize(count);
            for(int i = 0; i < count; i++)
            {
                scanmsg->ranges[i] = float(scan.distance_data.at(i)) / 1000.0f;
            }
        }

        pub->publish(*scanmsg);
    }
};

} // End of namespace


void ExitHandler(int sig)
{
    (void)sig;
    bea_power::to_exit = true;
    std::cout << "User terminated the node." << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bea_power::BEANode>();
    signal(SIGINT, ExitHandler);
    node->work_loop();
    //rclcpp::spin(node);
    rclcpp::shutdown();
}
