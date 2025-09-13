#include <arpa/inet.h>
#include <geometry_msgs/PoseStamped.h>
#include <iomanip>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string>
#include <sys/socket.h>
#include <tf/tf.h>
#include <thread>
#include <unistd.h>

class FCUCommandServer {
private:
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber status_sub_;

  // Debug subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gnss_sub_;

  int server_socket_;
  int client_socket_;
  bool client_connected_;
  std::mutex client_mutex_;

  std_msgs::Int16 cmd_;
  std_msgs::String status_msg_;

  // Debug data storage
  nav_msgs::Odometry latest_odom_;
  sensor_msgs::Imu latest_imu_;
  sensor_msgs::NavSatFix latest_gnss_;
  std::mutex debug_data_mutex_;
  bool odom_received_;
  bool imu_received_;
  bool gnss_received_;

public:
  FCUCommandServer()
      : client_connected_(false), odom_received_(false), imu_received_(false),
        gnss_received_(false) {
    command_pub_ = nh_.advertise<std_msgs::Int16>("/fcu_bridge/command", 100);
    status_pub_ = nh_.advertise<std_msgs::String>("/fcu_command/status", 100);
    status_sub_ = nh_.subscribe("/fcu_bridge/status", 100,
                                &FCUCommandServer::statusCallback, this);

    // Debug subscribers
    odom_sub_ = nh_.subscribe("/odom_global_001", 100,
                              &FCUCommandServer::odomCallback, this);
    imu_sub_ = nh_.subscribe("/imu_global_001", 100,
                             &FCUCommandServer::imuCallback, this);
    gnss_sub_ = nh_.subscribe("/gnss_global_001", 100,
                              &FCUCommandServer::gnssCallback, this);

    // Create TCP server socket
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0) {
      ROS_ERROR("Failed to create server socket");
      return;
    }

    // Set socket options
    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind socket
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8888);

    if (bind(server_socket_, (struct sockaddr *)&server_addr,
             sizeof(server_addr)) < 0) {
      ROS_ERROR("Failed to bind server socket");
      close(server_socket_);
      return;
    }

    // Listen for connections
    if (listen(server_socket_, 1) < 0) {
      ROS_ERROR("Failed to listen on server socket");
      close(server_socket_);
      return;
    }

    ROS_INFO("FCU Command Server started on port 8888");

    // Start server thread
    std::thread server_thread(&FCUCommandServer::serverLoop, this);
    server_thread.detach();
  }

  ~FCUCommandServer() {
    if (client_socket_ > 0) {
      close(client_socket_);
    }
    if (server_socket_ > 0) {
      close(server_socket_);
    }
  }

  void statusCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("Received status update: %s", msg->data.c_str());
    sendJsonResponse("status", msg->data);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(debug_data_mutex_);
    latest_odom_ = *msg;
    odom_received_ = true;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(debug_data_mutex_);
    latest_imu_ = *msg;
    imu_received_ = true;
  }

  void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(debug_data_mutex_);
    latest_gnss_ = *msg;
    gnss_received_ = true;
  }

  std::string createJsonResponse(const std::string &type,
                                 const std::string &message,
                                 const Json::Value &data = Json::Value()) {
    Json::Value response;
    response["type"] = type;
    response["message"] = message;
    response["timestamp"] = ros::Time::now().toSec();
    if (!data.isNull()) {
      response["data"] = data;
    }

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, response) + "\n";
  }

  void sendJsonResponse(const std::string &type, const std::string &message,
                        const Json::Value &data = Json::Value()) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_connected_) {
      std::string response = createJsonResponse(type, message, data);
      int bytes_sent =
          send(client_socket_, response.c_str(), response.length(), 0);
      if (bytes_sent > 0) {
        ROS_INFO("Sent JSON response to client: %s", message.c_str());
      } else {
        ROS_ERROR("Failed to send JSON response to client");
      }
    } else {
      ROS_WARN("Client not connected, cannot send JSON response");
    }
  }

  Json::Value getPositionData() {
    Json::Value data;
    std::lock_guard<std::mutex> lock(debug_data_mutex_);

    if (odom_received_) {
      data["world_position"]["x"] = latest_odom_.pose.pose.position.x;
      data["world_position"]["y"] = latest_odom_.pose.pose.position.y;
      data["world_position"]["z"] = latest_odom_.pose.pose.position.z;

      // Convert quaternion to euler angles
      tf::Quaternion q(latest_odom_.pose.pose.orientation.x,
                       latest_odom_.pose.pose.orientation.y,
                       latest_odom_.pose.pose.orientation.z,
                       latest_odom_.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      data["orientation"]["roll"] = roll;
      data["orientation"]["pitch"] = pitch;
      data["orientation"]["yaw"] = yaw;
      data["orientation"]["quaternion"]["x"] =
          latest_odom_.pose.pose.orientation.x;
      data["orientation"]["quaternion"]["y"] =
          latest_odom_.pose.pose.orientation.y;
      data["orientation"]["quaternion"]["z"] =
          latest_odom_.pose.pose.orientation.z;
      data["orientation"]["quaternion"]["w"] =
          latest_odom_.pose.pose.orientation.w;

      data["velocity"]["linear"]["x"] = latest_odom_.twist.twist.linear.x;
      data["velocity"]["linear"]["y"] = latest_odom_.twist.twist.linear.y;
      data["velocity"]["linear"]["z"] = latest_odom_.twist.twist.linear.z;
      data["velocity"]["angular"]["x"] = latest_odom_.twist.twist.angular.x;
      data["velocity"]["angular"]["y"] = latest_odom_.twist.twist.angular.y;
      data["velocity"]["angular"]["z"] = latest_odom_.twist.twist.angular.z;
    }

    if (imu_received_) {
      data["imu"]["linear_acceleration"]["x"] =
          latest_imu_.linear_acceleration.x;
      data["imu"]["linear_acceleration"]["y"] =
          latest_imu_.linear_acceleration.y;
      data["imu"]["linear_acceleration"]["z"] =
          latest_imu_.linear_acceleration.z;
      data["imu"]["angular_velocity"]["x"] = latest_imu_.angular_velocity.x;
      data["imu"]["angular_velocity"]["y"] = latest_imu_.angular_velocity.y;
      data["imu"]["angular_velocity"]["z"] = latest_imu_.angular_velocity.z;
    }

    if (gnss_received_) {
      data["gnss"]["latitude"] = latest_gnss_.latitude;
      data["gnss"]["longitude"] = latest_gnss_.longitude;
      data["gnss"]["altitude"] = latest_gnss_.altitude;
      data["gnss"]["status"] = latest_gnss_.status.status;
    }

    return data;
  }

  Json::Value getTopicInfo() {
    Json::Value topics;
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for (const auto &topic_info : topic_infos) {
      Json::Value topic;
      topic["name"] = topic_info.name;
      topic["type"] = topic_info.datatype;
      topics.append(topic);
    }

    return topics;
  }

  void serverLoop() {
    while (ros::ok()) {
      ROS_INFO("Waiting for client connection...");

      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);

      client_socket_ =
          accept(server_socket_, (struct sockaddr *)&client_addr, &client_len);
      if (client_socket_ < 0) {
        ROS_ERROR("Failed to accept client connection");
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(client_mutex_);
        client_connected_ = true;
      }

      ROS_INFO("Client connected from %s", inet_ntoa(client_addr.sin_addr));

      // Send welcome message
      Json::Value commands;
      commands["a"] = "unlock";
      commands["d"] = "lock";
      commands["t"] = "takeoff";
      commands["l"] = "land";
      commands["r"] = "run";
      commands["s"] = "stop";
      commands["1"] = "position_1";
      commands["2"] = "position_2";
      commands["3"] = "position_3";
      commands["4"] = "position_4";
      commands["p"] = "position_info";
      commands["i"] = "topic_info";
      commands["h"] = "help";
      commands["q"] = "quit";

      sendJsonResponse("welcome", "FCU Command Server Connected!", commands);

      // Handle client commands
      char buffer[256];
      while (ros::ok()) {
        int bytes_received =
            recv(client_socket_, buffer, sizeof(buffer) - 1, 0);
        if (bytes_received <= 0) {
          ROS_INFO("Client disconnected");
          break;
        }

        buffer[bytes_received] = '\0';
        std::string command(buffer);

        // Remove newline characters
        command.erase(std::remove(command.begin(), command.end(), '\n'),
                      command.end());
        command.erase(std::remove(command.begin(), command.end(), '\r'),
                      command.end());

        ROS_INFO("Received raw command: '%s' (length: %zu)", command.c_str(),
                 command.length());

        if (command.length() == 1) {
          ROS_INFO("Processing single character command: '%c'", command[0]);
          processCommand(command[0]);
        } else if (command == "quit" || command == "exit") {
          ROS_INFO("Client requested disconnect");
          sendJsonResponse("goodbye", "Goodbye!");
          break;
        } else {
          ROS_WARN("Invalid command format received: '%s' (length: %zu)",
                   command.c_str(), command.length());
          sendJsonResponse("error",
                           "Invalid command! Use single character commands.");
        }
      }

      {
        std::lock_guard<std::mutex> lock(client_mutex_);
        client_connected_ = false;
      }

      close(client_socket_);
    }
  }

  void processCommand(char cmd_char) {
    std::string command_name;
    Json::Value data;

    ROS_INFO("Received command: '%c' from client", cmd_char);

    switch (cmd_char) {
    case 'a':
      cmd_.data = 1;
      command_name = "Unlock";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case 'd':
      cmd_.data = 2;
      command_name = "Lock";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case 't':
      cmd_.data = 3;
      command_name = "Takeoff";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case 'l':
      cmd_.data = 4;
      command_name = "Land";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case 'r':
      cmd_.data = 5;
      command_name = "Run";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case 's':
      cmd_.data = 6;
      command_name = "Stop";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case '1':
      cmd_.data = 7;
      command_name = "Position 1";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case '2':
      cmd_.data = 8;
      command_name = "Position 2";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case '3':
      cmd_.data = 9;
      command_name = "Position 3";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case '4':
      cmd_.data = 10;
      command_name = "Position 4";
      command_pub_.publish(cmd_);
      data["command_id"] = cmd_.data;
      sendJsonResponse("command", command_name, data);
      ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(),
               cmd_.data);
      break;
    case 'p':
      command_name = "Position Info";
      data = getPositionData();
      sendJsonResponse("debug", command_name, data);
      ROS_INFO("Sending position info to client");
      break;
    case 'i':
      command_name = "Topic Info";
      data["topics"] = getTopicInfo();
      sendJsonResponse("debug", command_name, data);
      ROS_INFO("Sending topic info to client");
      break;
    case 'h': {
      command_name = "Help";
      Json::Value commands;
      commands["a"] = "unlock";
      commands["d"] = "lock";
      commands["t"] = "takeoff";
      commands["l"] = "land";
      commands["r"] = "run";
      commands["s"] = "stop";
      commands["1"] = "position_1";
      commands["2"] = "position_2";
      commands["3"] = "position_3";
      commands["4"] = "position_4";
      commands["p"] = "position_info";
      commands["i"] = "topic_info";
      commands["h"] = "help";
      commands["q"] = "quit";
      data["commands"] = commands;
      sendJsonResponse("help", command_name, data);
      ROS_INFO("Sending help to client");
      break;
    }
    default:
      sendJsonResponse("error", "Invalid command!");
      ROS_WARN("Invalid command received: '%c'", cmd_char);
      break;
    }
  }

  void run() { ros::spin(); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fcu_command_server");

  FCUCommandServer server;
  server.run();

  return 0;
}
