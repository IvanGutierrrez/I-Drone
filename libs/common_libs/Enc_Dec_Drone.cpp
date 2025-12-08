/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Drone.cpp                
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Enc_Dec_Drone.h"
#include "generated_proto/messages_pld.pb.h"
#include <boost/asio/detail/socket_ops.hpp>

namespace Enc_Dec_Drone {


    std::pair<Drone, std::unique_ptr<google::protobuf::Message>> decode_to_drone(const std::string& data)
    {
        WrapperDroneCommand wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {Drone::UNKNOWN, nullptr};
        }

        if (wrapper.has_command_message_normal()) {
            auto msg = std::make_unique<DroneCommandString>(wrapper.command_message_normal());
            return {Drone::COMMAND, std::move(msg)};
        }

        return {Drone::UNKNOWN, nullptr};
    }

    bool encode_command(const Struct_Drone::MessagePX4 &command, std::string &response)
    {
        DroneCommandString command_msg;
        command_msg.set_type_command(command.type);
        
        DroneCommandMision* mission_msg = command_msg.mutable_command();
        mission_msg->set_latitude_deg(command.mission_item.latitude_deg);
        mission_msg->set_longitude_deg(command.mission_item.longitude_deg);
        mission_msg->set_relative_altitude_m(command.mission_item.relative_altitude_m);
        mission_msg->set_speed_m_s(command.mission_item.speed_m_s);
        mission_msg->set_is_fly_through(command.mission_item.is_fly_through);
        mission_msg->set_gimbal_pitch_deg(command.mission_item.gimbal_pitch_deg);
        mission_msg->set_gimbal_yaw_deg(command.mission_item.gimbal_yaw_deg);
        
        switch (command.mission_item.camera_action) {
            case Struct_Drone::CameraAction::TakePhoto:
                mission_msg->set_camera_action(CAMERA_ACTION_TAKE_PHOTO);
                break;
            case Struct_Drone::CameraAction::StartPhotoInterval:
                mission_msg->set_camera_action(CAMERA_ACTION_START_PHOTO_INTERVAL);
                break;
            case Struct_Drone::CameraAction::StopPhotoInterval:
                mission_msg->set_camera_action(CAMERA_ACTION_STOP_PHOTO_INTERVAL);
                break;
            case Struct_Drone::CameraAction::StartVideo:
                mission_msg->set_camera_action(CAMERA_ACTION_START_VIDEO);
                break;
            case Struct_Drone::CameraAction::StopVideo:
                mission_msg->set_camera_action(CAMERA_ACTION_STOP_VIDEO);
                break;
            case Struct_Drone::CameraAction::StartPhotoDistance:
                mission_msg->set_camera_action(CAMERA_ACTION_START_PHOTO_DISTANCE);
                break;
            case Struct_Drone::CameraAction::StopPhotoDistance:
                mission_msg->set_camera_action(CAMERA_ACTION_STOP_PHOTO_DISTANCE);
                break;
            default:
                mission_msg->set_camera_action(CAMERA_ACTION_NONE);
                break;
        }
        
        WrapperDroneCommand wrapper;
        *(wrapper.mutable_command_message_normal()) = command_msg;

        std::string serialized_data;
        if (!wrapper.SerializeToString(&serialized_data))
            return false;

        uint32_t size = boost::asio::detail::socket_ops::host_to_network_long(
            static_cast<uint32_t>(serialized_data.size())
        );

        response.resize(4);
        std::memcpy(response.data(), &size, 4);
        response += serialized_data;

        return true;
    }

    bool decode_command(const DroneCommandString &msg, Struct_Drone::MessagePX4 &command)
    {
        try {
            command.type = msg.type_command();
            
            if (msg.has_command()) {
                const DroneCommandMision& mission_msg = msg.command();
                
                command.mission_item.latitude_deg = mission_msg.latitude_deg();
                command.mission_item.longitude_deg = mission_msg.longitude_deg();
                command.mission_item.relative_altitude_m = mission_msg.relative_altitude_m();
                command.mission_item.speed_m_s = mission_msg.speed_m_s();
                command.mission_item.is_fly_through = mission_msg.is_fly_through();
                command.mission_item.gimbal_pitch_deg = mission_msg.gimbal_pitch_deg();
                command.mission_item.gimbal_yaw_deg = mission_msg.gimbal_yaw_deg();
                
                // Convert CameraAction enum
                switch (mission_msg.camera_action()) {
                    case CAMERA_ACTION_TAKE_PHOTO:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::TakePhoto;
                        break;
                    case CAMERA_ACTION_START_PHOTO_INTERVAL:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::StartPhotoInterval;
                        break;
                    case CAMERA_ACTION_STOP_PHOTO_INTERVAL:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::StopPhotoInterval;
                        break;
                    case CAMERA_ACTION_START_VIDEO:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::StartVideo;
                        break;
                    case CAMERA_ACTION_STOP_VIDEO:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::StopVideo;
                        break;
                    case CAMERA_ACTION_START_PHOTO_DISTANCE:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::StartPhotoDistance;
                        break;
                    case CAMERA_ACTION_STOP_PHOTO_DISTANCE:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::StopPhotoDistance;
                        break;
                    default:
                        command.mission_item.camera_action = Struct_Drone::CameraAction::None;
                        break;
                }
            }
            
            return true;
        } catch (...) {
            return false;
        }
    }
    
    bool encode_status_drone(const Struct_Drone::Status &status, std::string &message)
    {
        WrapperDrone wrapper;

        Status* status_msg = wrapper.mutable_status();

        std::string state_std = to_string(status);
        if (state_std.empty())
            return false;

        status_msg->set_type_status(state_std);

        std::string serialized_data;
        if (!wrapper.SerializeToString(&serialized_data))
            return false;

        uint32_t size = boost::asio::detail::socket_ops::host_to_network_long(
            static_cast<uint32_t>(serialized_data.size())
        );

        message.resize(4);
        std::memcpy(message.data(), &size, 4);
        message += serialized_data;

        return true;
    }
};