
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/action/rotate_absolute.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <filesystem>

using namespace std::chrono_literals;

enum class CommandType {
    SetPen,
    Teleport,
    Move,
    Revolve,
    ENUM_SIZE
};

std::string kCommandTokens[] = {
    "spen",
    "tele",
    "move",
    "revl"
};

struct SetPenParam {
    SetPenParam(uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool is_off) : r(r), g(g), b(b), width(width), is_off(is_off) {}
    uint8_t r, g, b, width;
    bool is_off;
};

struct TeleportParam {
    TeleportParam(float x, float y) : x(x), y(y) {}
    float x, y;
};

struct MoveParam {
    MoveParam(float x, float y) : x(x), y(y) {}
    float x, y;
};

struct RevolveParam {
    RevolveParam(float origin_x, float origin_y, float times) : origin_x(origin_x), origin_y(origin_y), times(times) {}
    float origin_x, origin_y, times;
};

union CommandParams {
    SetPenParam set_pen;
    TeleportParam teleport;
    MoveParam move;
    RevolveParam revolve;
    CommandParams() {};
};


struct Command {
    CommandType type;
    CommandParams params;
};

std::vector<Command> ParseCommandsFromFile(const char* filename) {
    std::vector<Command> cmds;
    std::ifstream file(filename);
    std::string line;

    while(std::getline(file, line)) {

        std::stringstream sstream(line);
        std::string token;
        CommandType cmdtype = CommandType::ENUM_SIZE;
        CommandParams cmdparams;
        int i = 0;
        while(std::getline(sstream, token, ' ')) {
            if(i == 0) {
                if(token == kCommandTokens[static_cast<int>(CommandType::SetPen)]) {
                    cmdtype = CommandType::SetPen;
                } else if(token == kCommandTokens[static_cast<int>(CommandType::Teleport)]) {
                    cmdtype = CommandType::Teleport;
                } else if(token == kCommandTokens[static_cast<int>(CommandType::Move)]) {
                    cmdtype = CommandType::Move;
                } else if(token == kCommandTokens[static_cast<int>(CommandType::Revolve)]) {
                    cmdtype = CommandType::Revolve;
                } else {
                    break;
                }
            } else if (i > 0) {
                switch (cmdtype) {
                case CommandType::SetPen:
                    switch (i) {
                    case 1: cmdparams.set_pen.r = std::stoi(token); break;
                    case 2: cmdparams.set_pen.g = std::stoi(token); break;
                    case 3: cmdparams.set_pen.b = std::stoi(token); break;
                    case 4: cmdparams.set_pen.width = std::stoi(token); break;
                    case 5: cmdparams.set_pen.is_off = std::stoi(token); break;
                    } break;
                case CommandType::Teleport:
                    switch (i) {
                    case 1: cmdparams.teleport.x = std::stod(token); break;
                    case 2: cmdparams.teleport.y = std::stod(token); break;
                    } break;
                case CommandType::Move:
                    switch (i) {
                    case 1: cmdparams.move.x = std::stod(token); break;
                    case 2: cmdparams.move.y = std::stod(token); break;
                    } break;
                case CommandType::Revolve:
                    switch (i) {
                    case 1: cmdparams.revolve.origin_x = std::stod(token); break;
                    case 2: cmdparams.revolve.origin_y = std::stod(token); break;
                    case 3: cmdparams.revolve.times = std::stod(token); break;
                    } break;
                default:
                    break;
                }
            }
            i++;
        }
        if(cmdtype != CommandType::ENUM_SIZE) {
            cmds.push_back({cmdtype, cmdparams});
        }
    }
    return cmds;
}

class TurtleDraw {
public:
    TurtleDraw(std::shared_ptr<rclcpp::Node> nh);
    void Run();
private:
    std::string tdraw_path;
    turtlesim::msg::Pose pose;
    void pose_callback(const turtlesim::msg::Pose & msg) {
        pose = msg;
    }
    void rotate_abs_send_goal(float theta);
    std::vector<Command> commands;
    size_t command_idx;
    std::shared_ptr<rclcpp::Node> node_handle;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr setpen_client;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_abs_client;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_abs_client;
};


TurtleDraw::TurtleDraw(std::shared_ptr<rclcpp::Node> nh) : node_handle(nh) {
    
    node_handle->declare_parameter("filename", "");
    tdraw_path = node_handle->get_parameter("filename").as_string();
    if(!std::filesystem::exists(tdraw_path)) {
        RCLCPP_ERROR(node_handle->get_logger(), "Invalid filename parameter. To use file, run: ros2 run turtle_draw_stuff turtle_draw --ros-args -p \"filename:=[ENTER FILENAME HERE]\"");
        exit(0);
    }
    commands = ParseCommandsFromFile(tdraw_path.c_str());
    command_idx = 0;
    auto pose_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    pose_sub = nh->create_subscription<turtlesim::msg::Pose>("turtle1/pose", pose_qos, std::bind(&TurtleDraw::pose_callback, this, std::placeholders::_1));
    setpen_client = nh->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    teleport_abs_client = nh->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
    twist_pub = nh->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    rotate_abs_client = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(nh, "turtle1/rotate_absolute");
}

void TurtleDraw::rotate_abs_send_goal(float theta) {
    using namespace std::placeholders;
    if(!rotate_abs_client->wait_for_action_server()) {
        RCLCPP_ERROR(node_handle->get_logger(), "Action server not available after waiting");
        exit(0);
    }
    auto goal_msg = turtlesim::action::RotateAbsolute::Goal();
    goal_msg.theta = theta;
    auto send_goal_future = rotate_abs_client->async_send_goal(goal_msg);
    if(rclcpp::spin_until_future_complete(node_handle, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_handle->get_logger(), "Failed to send goal");
        exit(1);
    }
    auto goal_handle = send_goal_future.get();
    if(!goal_handle) {
        RCLCPP_ERROR(node_handle->get_logger(), "Goal was rejected by server");
        exit(1);
    }
    auto result_future = rotate_abs_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_handle, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_handle->get_logger(), "Failed to get result");
        exit(1);
    }
}

void TurtleDraw::Run() {
    bool running = true;
    rotate_abs_send_goal(0);
    while(running) {
        rclcpp::spin_some(node_handle);
        if(command_idx >= commands.size()) {
            running = false;
            break;
        }
        switch (commands[command_idx].type) {
        case CommandType::SetPen: {
            auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
            request->r = commands[command_idx].params.set_pen.r;                        
            request->g = commands[command_idx].params.set_pen.g;
            request->b = commands[command_idx].params.set_pen.b;
            request->width = commands[command_idx].params.set_pen.width;
            request->off = commands[command_idx].params.set_pen.is_off;
            while (!setpen_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    exit(0);
                }
            }
            rclcpp::spin_until_future_complete(node_handle, setpen_client->async_send_request(request), 1s);
            break;
        }
        case CommandType::Teleport: {
            auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            request->x = commands[command_idx].params.teleport.x;
            request->y = commands[command_idx].params.teleport.y;
            printf("%f, %f\n", request->x, request->y);

            while (!teleport_abs_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    exit(0);
                }
            }
            rclcpp::spin_until_future_complete(node_handle, teleport_abs_client->async_send_request(request), 1s);
            break;
        }
        case CommandType::Move: {
            float next_bot_angle = atan2(
                commands[command_idx].params.move.y - pose.y ,
                commands[command_idx].params.move.x - pose.x
            );
            auto the_scoot = geometry_msgs::msg::Twist();
            the_scoot.linear.x = commands[command_idx].params.move.x - pose.x;
            the_scoot.linear.y = commands[command_idx].params.move.y - pose.y;
            twist_pub->publish(the_scoot);
            rclcpp::sleep_for(1100ms);
            break;
        }
        case CommandType::Revolve: {
            printf("%f, %f\n", pose.x - commands[command_idx].params.revolve.origin_x, pose.y - commands[command_idx].params.revolve.origin_y);

            float next_bot_angle = atan2(
                pose.y - commands[command_idx].params.revolve.origin_y,
                pose.x - commands[command_idx].params.revolve.origin_x
            ) - (M_PI/2 * copysign(1.0, commands[command_idx].params.revolve.times));
            auto the_twist = geometry_msgs::msg::Twist();
            the_twist.angular.z = next_bot_angle - pose.theta;
            twist_pub->publish(the_twist);
            rclcpp::sleep_for(1100ms);
            float radius = hypot(
                pose.x - commands[command_idx].params.revolve.origin_x,
                pose.y - commands[command_idx].params.revolve.origin_y
            ) * abs(commands[command_idx].params.revolve.times);
            auto the_revolution = geometry_msgs::msg::Twist();
            the_revolution.linear.x = 2 * M_PI * radius;
            the_revolution.angular.z = -2 * M_PI * commands[command_idx].params.revolve.times;//-(2 * M_PI * commands[command_idx].params.revolve.times);
            twist_pub->publish(the_revolution);
            rclcpp::sleep_for(1100ms);
            rotate_abs_send_goal(0);
            break;
        }
        default:
            break;
        }
        printf("Step %zu done\n", command_idx);
        command_idx++;
    }
}



void quit(int sig) {
    (void)sig;
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    signal(SIGINT, quit);
    try {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("turtle_draw_stuff");
    TurtleDraw turtle_draw_stuff(node);
    turtle_draw_stuff.Run();

    } catch (std::runtime_error &e) {
        printf("OOPS!!!: %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
