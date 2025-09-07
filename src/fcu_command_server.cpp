#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>

class FCUCommandServer {
private:
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
    ros::Publisher status_pub_;
    ros::Subscriber status_sub_;
    
    int server_socket_;
    int client_socket_;
    bool client_connected_;
    std::mutex client_mutex_;
    
    std_msgs::Int16 cmd_;
    std_msgs::String status_msg_;

public:
    FCUCommandServer() : client_connected_(false) {
        command_pub_ = nh_.advertise<std_msgs::Int16>("/fcu_bridge/command", 100);
        status_pub_ = nh_.advertise<std_msgs::String>("/fcu_command/status", 100);
        status_sub_ = nh_.subscribe("/fcu_bridge/status", 100, &FCUCommandServer::statusCallback, this);
        
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
        
        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
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
    
    void statusCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Received status update: %s", msg->data.c_str());
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_connected_) {
            std::string response = "STATUS:" + msg->data + "\n";
            int bytes_sent = send(client_socket_, response.c_str(), response.length(), 0);
            if (bytes_sent > 0) {
                ROS_INFO("Sent status to client: %s", msg->data.c_str());
            } else {
                ROS_ERROR("Failed to send status to client");
            }
        } else {
            ROS_WARN("Client not connected, cannot send status update");
        }
    }
    
    void serverLoop() {
        while (ros::ok()) {
            ROS_INFO("Waiting for client connection...");
            
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            client_socket_ = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
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
            std::string welcome = "FCU Command Server Connected!\nCommands: a=unlock, d=lock, t=takeoff, l=land, r=run, s=stop, 1-4=positions\n";
            send(client_socket_, welcome.c_str(), welcome.length(), 0);
            
            // Handle client commands
            char buffer[256];
            while (ros::ok()) {
                int bytes_received = recv(client_socket_, buffer, sizeof(buffer) - 1, 0);
                if (bytes_received <= 0) {
                    ROS_INFO("Client disconnected");
                    break;
                }
                
                buffer[bytes_received] = '\0';
                std::string command(buffer);
                
                // Remove newline characters
                command.erase(std::remove(command.begin(), command.end(), '\n'), command.end());
                command.erase(std::remove(command.begin(), command.end(), '\r'), command.end());
                
                ROS_INFO("Received raw command: '%s' (length: %zu)", command.c_str(), command.length());
                
                if (command.length() == 1) {
                    ROS_INFO("Processing single character command: '%c'", command[0]);
                    processCommand(command[0]);
                } else if (command == "quit" || command == "exit") {
                    ROS_INFO("Client requested disconnect");
                    std::string response = "Goodbye!\n";
                    send(client_socket_, response.c_str(), response.length(), 0);
                    break;
                } else {
                    ROS_WARN("Invalid command format received: '%s' (length: %zu)", command.c_str(), command.length());
                    std::string response = "Invalid command! Use single character commands.\n";
                    send(client_socket_, response.c_str(), response.length(), 0);
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
        std::string response;
        std::string command_name;
        
        ROS_INFO("Received command: '%c' from client", cmd_char);
        
        switch(cmd_char) {
            case 'a':
                cmd_.data = 1;
                command_name = "Unlock";
                command_pub_.publish(cmd_);
                response = "Command: Unlock\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case 'd':
                cmd_.data = 2;
                command_name = "Lock";
                command_pub_.publish(cmd_);
                response = "Command: Lock\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case 't':
                cmd_.data = 3;
                command_name = "Takeoff";
                command_pub_.publish(cmd_);
                response = "Command: Takeoff\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case 'l':
                cmd_.data = 4;
                command_name = "Land";
                command_pub_.publish(cmd_);
                response = "Command: Land\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case 'r':
                cmd_.data = 5;
                command_name = "Run";
                command_pub_.publish(cmd_);
                response = "Command: Run\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case 's':
                cmd_.data = 6;
                command_name = "Stop";
                command_pub_.publish(cmd_);
                response = "Command: Stop\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case '1':
                cmd_.data = 7;
                command_name = "Position 1";
                command_pub_.publish(cmd_);
                response = "Command: Position 1\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case '2':
                cmd_.data = 8;
                command_name = "Position 2";
                command_pub_.publish(cmd_);
                response = "Command: Position 2\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case '3':
                cmd_.data = 9;
                command_name = "Position 3";
                command_pub_.publish(cmd_);
                response = "Command: Position 3\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            case '4':
                cmd_.data = 10;
                command_name = "Position 4";
                command_pub_.publish(cmd_);
                response = "Command: Position 4\n";
                ROS_INFO("Executing command: %s (ID: %d)", command_name.c_str(), cmd_.data);
                break;
            default:
                response = "Invalid command!\n";
                ROS_WARN("Invalid command received: '%c'", cmd_char);
                break;
        }
        
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_connected_) {
            int bytes_sent = send(client_socket_, response.c_str(), response.length(), 0);
            if (bytes_sent > 0) {
                ROS_INFO("Sent response to client: %s", response.c_str());
            } else {
                ROS_ERROR("Failed to send response to client");
            }
        } else {
            ROS_WARN("Client not connected, cannot send response");
        }
    }
    
    void run() {
        ros::spin();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "fcu_command_server");
    
    FCUCommandServer server;
    server.run();
    
    return 0;
}
