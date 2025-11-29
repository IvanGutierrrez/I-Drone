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

        if (wrapper.has_command_message()) {
            auto msg = std::make_unique<DroneCommand>(wrapper.command_message());
            return {Drone::COMMAND, std::move(msg)};
        }

        return {Drone::UNKNOWN, nullptr};
    }

    bool encode_command(const std::string &command, std::string &response)
    {
        DroneCommand command_msg;
        command_msg.set_command(command);

        WrapperDroneCommand wrapper;
        *(wrapper.mutable_command_message()) = command_msg;

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

    bool decode_command(const DroneCommand &msg, std::string &command)
    {
        try {
            command = msg.command();
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