#include <bits/stdc++.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
using namespace std;
#define LINHA 29
#define COLUNA 29

class AlgoritmoParte2 : public rclcpp::Node
{
public:
    AlgoritmoParte2()
    : Node("algoritmo_parte_2")
    {
        move_robot_client = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");

        subscriber = this->create_subscription<cg_interfaces::msg::RobotSensors>("/culling_games/robot_sensors", 10, topic_callback)
    }

    int move(char direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();

        switch(direction) {
            case 'u': request->direction = "up"; break;
            case 'd': request->direction = "down"; break;
            case 'l': request->direction = "left"; break;
            case 'r': request->direction = "right"; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Direção inválida");
                return -1;
        }

        if (!move_robot_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Serviço move_command não disponível");
            return -1;
        }

        auto future = move_robot_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            switch(direction) {
            case 'u': RCLCPP_INFO(this->get_logger(), "Movido para cima!"); break;
            case 'd': RCLCPP_INFO(this->get_logger(), "Movido para baixo!"); break;
            case 'l': RCLCPP_INFO(this->get_logger(), "Movido para esquerda!"); break;
            case 'r': RCLCPP_INFO(this->get_logger(), "Movido para direita!"); break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Direção inválida");
                return -1;
            }
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Falha ao mover");
            return -1;
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_robot_client;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

};