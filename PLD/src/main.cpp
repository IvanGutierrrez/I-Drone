/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : main.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <boost/asio.hpp>
#include "common_libs/Server.h"
#include "generated_proto/messages_pld.pb.h"
#include "common_libs/Enc_Dec_Drone.h"
#include "common_libs/Enc_Dec_PLD.h"
#include <memory>
#include <vector>
#include <functional>

using boost::asio::ip::tcp;

int main() {
    boost::asio::io_context io;

    // Un solo servidor PLD esperando la conexión del módulo Drone
    tcp::endpoint endpoint(boost::asio::ip::make_address("0.0.0.0"), 12345);
    auto server = std::make_shared<Server>(io);

    Server::handlers handlers;
    handlers.call_error = [](const boost::system::error_code& ec, const Type_Error &type){
        std::string log;
        switch (type){
        case Type_Error::CONNECTING:
            log = "Error connecting to Drone";
            break;
        case Type_Error::READING:
            log = "Error while reading a message from Drone";
            break;
        case Type_Error::SENDING:
            log = "Error while sending a message to Drone";
            break;
        default:
            log = "Unknown error communicating with Drone";
            break;
        }
        std::cout << log << ": " << ec.what() << std::endl;
    };
    handlers.call_connect = []() {
        std::cout << "Drone module connected" << std::endl;
    };
    handlers.call_message = [](const std::string& msg){
        std::cout << "Message received from Drone" << std::endl;
        std::string data = msg.substr(4);

        auto [type, decoded_msg] = Enc_Dec_PLD::decode_from_drone(data);
        if (type == Enc_Dec_PLD::PLD::STATUS_DRONE) {
            auto my_msg = dynamic_cast<Status*>(decoded_msg.get());
            if (my_msg) {
                std::cout << "Drone status: " << my_msg->type_status() << std::endl;
            }
        } else {
            std::cout << "Unknown message type received" << std::endl;
        }
    };

    server->set_handlers(handlers);
    server->start_listening(endpoint);

    // Timer para enviar mensajes a ambos drones a la vez
    auto timer = std::make_shared<boost::asio::steady_timer>(io, std::chrono::seconds(1));

    int tick = 0;
    std::function<void()> send_loop;
    send_loop = [&]() {
        std::cout << tick << std::endl;
        tick++;

        auto send_command = [&](const Struct_Drone::MessagePX4 &px4_msg){
            std::string msg;
            if (!Enc_Dec_Drone::encode_command(px4_msg, msg)) {
                std::cout << "Error encoding mission message" << std::endl;
                return;
            }
            server->deliver(msg);
        };

        // Misión para Drone 0 (va al este)
        if (tick == 10) {
            std::cout << "Sending waypoint 1 to Drone 0" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 40.416775;
            mission_msg.longitude_deg = -3.703790 + 0.00002; // 2 metros al este
            mission_msg.relative_altitude_m = 10.0;
            mission_msg.speed_m_s = 2.0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "START";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }
        // Misión para Drone 1 (va al oeste)
        else if (tick == 11) {
            std::cout << "Sending waypoint 1 to Drone 1" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 40.416775;
            mission_msg.longitude_deg = -3.703790 - 0.00002; // 2 metros al oeste
            mission_msg.relative_altitude_m = 10.0;
            mission_msg.speed_m_s = 2.0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "START";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }
        // Waypoint 2 para Drone 0
        else if (tick == 12) {
            std::cout << "Sending waypoint 2 to Drone 0" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 40.416775 + 0.00002; // norte
            mission_msg.longitude_deg = -3.703790 + 0.00002;
            mission_msg.relative_altitude_m = 10.0;
            mission_msg.speed_m_s = 2.0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }
        // Waypoint 2 para Drone 1
        else if (tick == 13) {
            std::cout << "Sending waypoint 2 to Drone 1" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 40.416775 + 0.00002; // norte
            mission_msg.longitude_deg = -3.703790 - 0.00002;
            mission_msg.relative_altitude_m = 10.0;
            mission_msg.speed_m_s = 2.0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }
        // Waypoint final para Drone 0
        else if (tick == 14) {
            std::cout << "Sending waypoint 3 to Drone 0" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 40.416775 + 0.00002;
            mission_msg.longitude_deg = -3.703790;
            mission_msg.relative_altitude_m = 10.0;
            mission_msg.speed_m_s = 2.0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "FINISH";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }
        // Waypoint final para Drone 1
        else if (tick == 15) {
            std::cout << "Sending waypoint 3 to Drone 1" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 40.416775 + 0.00002;
            mission_msg.longitude_deg = -3.703790;
            mission_msg.relative_altitude_m = 10.0;
            mission_msg.speed_m_s = 2.0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "FINISH";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }
        // Comando para iniciar todos los drones simultáneamente
        else if (tick == 16) {
            std::cout << "Sending START_ALL signal to begin all missions" << std::endl;
            Struct_Drone::MessagePX4 px4_msg;
            Struct_Drone::MissionItem mission_msg;
            mission_msg.latitude_deg = 0;
            mission_msg.longitude_deg = 0;
            mission_msg.relative_altitude_m = 0;
            mission_msg.speed_m_s = 0;
            mission_msg.is_fly_through = false;
            mission_msg.gimbal_pitch_deg = 0.0;
            mission_msg.gimbal_yaw_deg = 0.0;
            mission_msg.camera_action = Struct_Drone::CameraAction::None;
            px4_msg.type = "START_ALL";
            px4_msg.mission_item = mission_msg;
            send_command(px4_msg);
        }

        timer->expires_after(std::chrono::seconds(1));
        timer->async_wait([&](const boost::system::error_code& ec){
            if (!ec) send_loop();
        });
    };

    send_loop();
    io.run();
    return 0;
}