#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <s21c_receive_msg/msg/distance.hpp>  // 包含自定义消息头文件
#include <s21c_receive_msg/msg/stp23.hpp>     // 包含自定义STP23消息头文件
#include <s21c_receive_msg/msg/lidar.hpp>     // 包含自定义STP23消息头文件
#include <vector>
#include <array>
#include <map>
#include <string>

// 超声波数据接收处理和话题发布
void ultrasonic_data_deal(const std::vector<uint8_t>& buffer, rclcpp::Node::SharedPtr nh) {
    // 检查帧头和帧尾
    if (buffer[0] == 0xFD && buffer[buffer.size() - 1] == 0xDF) {
        // 计算校验和
        uint8_t checksum = 0;
        for (size_t i = 0; i < buffer.size() - 2; i++) {
            checksum += buffer[i];
        }
        if (checksum == buffer[buffer.size() - 2]) {
            // 解析数据
            s21c_receive_msg::msg::Distance msg;
            msg.a_distance = ((buffer[1] << 8) | buffer[2]) / 1000.0;
            msg.b_distance = ((buffer[3] << 8) | buffer[4]) / 1000.0;
            msg.c_distance = ((buffer[5] << 8) | buffer[6]) / 1000.0;
            msg.d_distance = ((buffer[7] << 8) | buffer[8]) / 1000.0;
            msg.e_distance = ((buffer[9] << 8) | buffer[10]) / 1000.0;
            msg.f_distance = ((buffer[11] << 8) | buffer[12]) / 1000.0;

            // 创建发布者
            static std::map<std::string, rclcpp::Publisher<s21c_receive_msg::msg::Distance>::SharedPtr> publishers;
            std::string topic_name = "distance";
            if (publishers.find(topic_name) == publishers.end()) {
                publishers[topic_name] = nh->create_publisher<s21c_receive_msg::msg::Distance>(topic_name, 10);
            }
            // 发布数据
            publishers[topic_name]->publish(msg);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("serial_listener"), "Checksum error in ultrasonic data");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("serial_listener"), "Invalid frame header or footer in ultrasonic data");
    }
}

// STP23数据接收处理和话题发布
void STP23_data_deal(const std::vector<uint8_t>& buffer, rclcpp::Node::SharedPtr nh) {
    // 检查帧头和帧尾
    if (buffer[0] == 0x7B && buffer[buffer.size() - 1] == 0x7D) {
        // 计算校验和--异或校验
        uint8_t checksum = 0;
        for (size_t i = 0; i < buffer.size() - 2; i++) {
            checksum ^= buffer[i];
        }
        if (checksum == buffer[buffer.size() - 2]) {
            // 解析数据
            s21c_receive_msg::msg::STP23 msg;
            msg.distance_1 = (buffer[1] << 8) | buffer[2];
            msg.distance_2 = (buffer[3] << 8) | buffer[4];
            msg.distance_3 = (buffer[5] << 8) | buffer[6];
            msg.distance_4 = (buffer[7] << 8) | buffer[8];

            // 创建发布者
            static std::map<std::string, rclcpp::Publisher<s21c_receive_msg::msg::STP23>::SharedPtr> publishers;
            std::string topic_name = "STP23";
            if (publishers.find(topic_name) == publishers.end()) {
                publishers[topic_name] = nh->create_publisher<s21c_receive_msg::msg::STP23>(topic_name, 10);
            }

            // 发布数据
            publishers[topic_name]->publish(msg);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("serial_listener"), "Checksum error");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("serial_listener"), "Invalid frame header or footer");
    }
}

void LD14P_revicer(uint8_t byte, std::vector<uint8_t>& buffer, rclcpp::Node::SharedPtr nh) {
    static bool frame_found = false;
    static rclcpp::Publisher<s21c_receive_msg::msg::Lidar>::SharedPtr publisher;
    static s21c_receive_msg::msg::Lidar msg;  // 静态消息变量

    // 雷达的帧头
    const uint8_t frame_heads[4][8] = {
        {0x01, 0x01, 0x01, 0x01, 0x0A, 0x0A, 0x01, 0x01},  // 雷达1角度帧头
        {0x01, 0x01, 0x01, 0x01, 0x0B, 0x0B, 0x01, 0x01},  // 雷达2角度帧头
        {0x01, 0x01, 0x01, 0x01, 0x0C, 0x0C, 0x01, 0x01},  // 雷达3角度帧头
        {0x01, 0x01, 0x01, 0x01, 0x0D, 0x0D, 0x01, 0x01}   // 雷达4角度帧头
    };
    // 雷达的帧尾
    const uint8_t frame_tail[8] = {0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D};

    buffer.push_back(byte);

    if (!frame_found) { // 帧头未找到
        // 检查缓冲区中的数据是否匹配任何一个帧头
        for (int i = 0; i < 4; i++) {
            if (buffer.size() >= 8 && std::equal(frame_heads[i], frame_heads[i] + 8, buffer.begin())) {
                frame_found = true; // 匹配到帧头
                return;
            }
        }
        // 如果缓冲区中的数据超过8个字节且没有匹配到任何帧头，移除第一个字节，继续检查
        if (buffer.size() >= 8) {
            buffer.erase(buffer.begin());
        }
    } else {
        // 两帧数据的总长度
        size_t total_length = 2896 + 1456;

        // 检查是否接收到完整的一帧数据
        if (buffer.size() >= total_length) {
            // 检查帧尾
            if (std::equal(frame_tail, frame_tail + 8, buffer.end() - 8)) {
                // 处理数据
                int lidar_id = (buffer[4] == 0x0A) ? 1 : (buffer[4] == 0x0B) ? 2 : (buffer[4] == 0x0C) ? 3 : 4;

                // 解析角度数据
                for (int i = 0; i < 720; i++) {
                    uint32_t raw_angle = 
                        (buffer[8 + i * 4 + 0] << 24) |
                        (buffer[8 + i * 4 + 1] << 16) |
                        (buffer[8 + i * 4 + 2] << 8)  |
                        (buffer[8 + i * 4 + 3]);
                    float angle = static_cast<float>(raw_angle) / 1000000.0;  // 转换回原始角度值
                    msg.angles[i] = angle;
                }

                // 解析距离数据
                for (int i = 0; i < 720; i++) {
                    uint16_t distance = 
                        (buffer[2904 + i * 2 + 0] << 8) |  // 2904 = 8 + 2896 - 8
                        (buffer[2904 + i * 2 + 1]);
                    msg.distances[i] = distance;
                }

                // 设置相应的标志位
                switch (lidar_id) {
                    case 1: msg.has_lidar_1 = true; break;
                    case 2: msg.has_lidar_2 = true; break;
                    case 3: msg.has_lidar_3 = true; break;
                    case 4: msg.has_lidar_4 = true; break;
                }

                // 打印角度和对应的距离
                // for (int i = 0; i < 720; i++) {
                //     RCLCPP_INFO(nh->get_logger(), "Angle[%d]: %f, Distance[%d]: %d", i, msg.angles[i], i, msg.distances[i]);
                // }

                // 创建发布者
                if (!publisher) {
                    publisher = nh->create_publisher<s21c_receive_msg::msg::Lidar>("lidar_data", 10);
                }

                // 发布数据
                publisher->publish(msg);

                // 清空消息内容，准备接收下一帧数据
                msg.has_lidar_1 = false;
                msg.has_lidar_2 = false;
                msg.has_lidar_3 = false;
                msg.has_lidar_4 = false;
                for (int i = 0; i < 720; i++) {
                    msg.angles[i] = 0.0;
                    msg.distances[i] = 0;
                }

                buffer.clear();  // 清空缓冲区
                frame_found = false; // 重新开始匹配
            } else {
                buffer.clear();  // 清空缓冲区，重新开始匹配
                frame_found = false; // 重新开始匹配
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("serial_listener");

    // 读取module_n参数
    int module_n = 0;
    nh->declare_parameter("module_n", 0);
    nh->get_parameter("module_n", module_n);
    RCLCPP_INFO(nh->get_logger(), "Module parameter (initial): %d", module_n);

    // 根据module_n参数输出当前设置的模块
    if (module_n == 0) {
        RCLCPP_INFO(nh->get_logger(), "Current module: Ultrasonic, Distance_data (unit: m)");
    } else if (module_n == 1) {
        RCLCPP_INFO(nh->get_logger(), "Current module: STP23/STP23L, Distance_data (unit: mm)");
    } else if (module_n == 2) {
        RCLCPP_INFO(nh->get_logger(), "Current module: LD14P, < Angle_data , Distance_data(unit: mm) >");
    } else {
        RCLCPP_WARN(nh->get_logger(), "Invalid module_n value: %d", module_n);
        return -1;
    }

    // 设置串口参数
    std::string port = "/dev/ttyCH343USB0";
    unsigned long baudrate = 115200;
    if (module_n == 2) {
        baudrate = 921600;
    }

    // 打开串口，并设置数据位、停止位、校验位等参数
    serial::Serial ser(port, baudrate, serial::Timeout::simpleTimeout(1000), 
                       serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none);
    if (!ser.isOpen()) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to open serial port %s", port.c_str());
        return -1;
    }
    RCLCPP_INFO(nh->get_logger(), "Serial port %s opened successfully", port.c_str());

    // 主循环
    std::vector<uint8_t> buffer;
    buffer.reserve(4500);  // 预分配足够大的缓冲区
    bool frame_start_found = false;  // 标记是否找到帧头

    while (rclcpp::ok()) {
        if (ser.available()) {
            uint8_t byte;
            size_t bytes_read = ser.read(&byte, 1);  // 逐字节读取
            if (bytes_read == 1) {
                if (module_n == 2) {
                    LD14P_revicer(byte, buffer, nh); // 读取LD14P模块数据
                } else {
                    // 其他模块的处理逻辑（stp23/stp23l/超声波）
                    if (!frame_start_found) {
                        if (module_n == 0 && byte == 0xFD) {
                            frame_start_found = true;
                            buffer.push_back(byte);
                        } else if (module_n == 1 && byte == 0x7B) {
                            frame_start_found = true;
                            buffer.push_back(byte);
                        }
                    } else {
                        buffer.push_back(byte);
                        // 检查是否达到预期的帧长度
                        if (module_n == 0 && buffer.size() == 19) {
                            ultrasonic_data_deal(buffer, nh);
                            buffer.clear();
                            frame_start_found = false;
                        } else if (module_n == 1 && buffer.size() == 11) {
                            STP23_data_deal(buffer, nh);
                            buffer.clear();
                            frame_start_found = false;
                        }
                    }
                }
            } else {
                RCLCPP_WARN(nh->get_logger(), "Failed to read byte from serial port");
            }
        }
        rclcpp::spin_some(nh);  // 处理ROS事件
    }

    rclcpp::shutdown();
    return 0;
}