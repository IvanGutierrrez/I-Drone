/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_PLD.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Enc_Dec_PLD.h"
#include <boost/asio/detail/socket_ops.hpp>
#include "structs/Structs_Planner.h"
#include "Enc_Dec_Planner.h"

namespace Enc_Dec_PLD {

    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_planner(const std::string& data)
    {
        WrapperPlanner wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {PLD::UNKNOWN, nullptr};
        }

        if (wrapper.has_planner_response()) {
            auto msg = std::make_unique<PlannerResponseList>(wrapper.planner_response());
            return {PLD::Planner_RESPONSE, std::move(msg)};
        }

        if (wrapper.has_status()) {
            auto msg = std::make_unique<Status>(wrapper.status());
            return {PLD::STATUS_Planner, std::move(msg)};
        }

        return {PLD::UNKNOWN, nullptr};
    }

    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_drone(const std::string& data)
    {
        WrapperDrone wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {PLD::UNKNOWN, nullptr};
        }

        if (wrapper.has_status()) {
            auto msg = std::make_unique<Status>(wrapper.status());
            return {PLD::STATUS_DRONE, std::move(msg)};
        }

        return {PLD::UNKNOWN, nullptr};
    }

    std::pair<PLD, std::unique_ptr<google::protobuf::Message>> decode_from_client(const std::string& data)
    {
        WrapperFromClient wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {PLD::UNKNOWN, nullptr};
        }

        if (wrapper.has_config()) {
            auto msg = std::make_unique<Config_mission>(wrapper.config());
            return {PLD::CONFIG_MISSION, std::move(msg)};
        }

        return {PLD::UNKNOWN, nullptr};
    }

    bool encode_planner_response(const std::vector<std::vector<Struct_Planner::Coordinate>> &result, std::string &msg)
    {
        PlannerResponseList dron_proto;

        for (const auto &path : result) {
            PlannerResponse* drone_msg = dron_proto.add_items();

            for (const auto &coord : path) {
                drone_msg->add_lon(coord.lon);
                drone_msg->add_lat(coord.lat);
            }
        }

        WrapperPlanner wrapper;
        *(wrapper.mutable_planner_response()) = dron_proto;

        std::string serialized_data;
        if (!wrapper.SerializeToString(&serialized_data))
            return false;

        uint32_t size = boost::asio::detail::socket_ops::host_to_network_long(
            static_cast<uint32_t>(serialized_data.size())
        );

        msg.resize(4);
        std::memcpy(msg.data(), &size, 4);
        msg += serialized_data;

        return true;
    }

    bool decode_planner_response(const PlannerResponseList &proto, std::vector<std::vector<Struct_Planner::Coordinate>> &result)
    {
        result.clear();
        result.reserve(proto.items().size());

        for (int d = 0; d < proto.items().size(); ++d)
        {
            const PlannerResponse &msg = proto.items(d);

            if (msg.lon_size() != msg.lat_size())
                return false;

            std::vector<Struct_Planner::Coordinate> drone_path;
            drone_path.reserve(msg.lon_size());

            for (int i = 0; i < msg.lon_size(); ++i)
            {
                drone_path.emplace_back(
                    msg.lon(i),
                    msg.lat(i)
                );
            }

            result.push_back(std::move(drone_path));
        }

        return true;
    }

    bool encode_status_planner(const Struct_Planner::Status &status, std::string &message)
    {
        WrapperPlanner wrapper;

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

    bool encode_status_pld(const Structs_PLD::Status &status, std::string &message)
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

    bool encode_config_mission(const Structs_PLD::Config_mission &config, std::string &message)
    {
        WrapperFromClient wrapper;
        Config_mission* config_msg = wrapper.mutable_config();

        std::string planner_msg_str;
        if (!Enc_Dec_Planner::encode_config_message(
            config.planner_info.signal_server_config,
            config.planner_info.dron_data,
            planner_msg_str)) {
            return false;
        }

        PlannerMessage planner_msg;
        if (!planner_msg.ParseFromString(planner_msg_str.substr(4))) {
            return false;
        }
        
        config_msg->mutable_planner_config()->CopyFrom(planner_msg);

        Info_Module* info_planner = config_msg->mutable_info_planner();
        info_planner->set_docker_name(config.planner_module_data.docker_name);
        info_planner->set_docker_file(config.planner_module_data.docker_file);
        info_planner->set_module_ip(config.planner_module_data.module_ip);
        info_planner->set_ssh_ip(config.planner_module_data.ssh_ip);
        info_planner->set_port(config.planner_module_data.port);
        info_planner->set_user(config.planner_module_data.user);
        info_planner->set_key(config.planner_module_data.key);

        Info_Module* info_drone = config_msg->mutable_info_drone();
        info_drone->set_docker_name(config.drone_module_data.docker_name);
        info_drone->set_docker_file(config.drone_module_data.docker_file);
        info_drone->set_module_ip(config.drone_module_data.module_ip);
        info_drone->set_ssh_ip(config.drone_module_data.ssh_ip);
        info_drone->set_port(config.drone_module_data.port);
        info_drone->set_user(config.drone_module_data.user);
        info_drone->set_key(config.drone_module_data.key);

        config_msg->set_drone_sim(config.drone_sim);

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

    bool decode_config_mission(const Config_mission &proto, Structs_PLD::Config_mission &config)
    {
        config.clear();

        const PlannerMessage& planner_msg = proto.planner_config();
        
        if (!Enc_Dec_Planner::decode_signal_server(planner_msg.signal_server_config(), config.planner_info.signal_server_config)) {
            return false;
        }
        
        if (!Enc_Dec_Planner::decode_drone_data(planner_msg.drone_data(), config.planner_info.dron_data)) {
            return false;
        }

        const Info_Module& info_planner = proto.info_planner();
        config.planner_module_data.docker_name = info_planner.docker_name();
        config.planner_module_data.docker_file = info_planner.docker_file();
        config.planner_module_data.module_ip = info_planner.module_ip();
        config.planner_module_data.ssh_ip = info_planner.ssh_ip();
        config.planner_module_data.port = info_planner.port();
        config.planner_module_data.user = info_planner.user();
        config.planner_module_data.key = info_planner.key();

        const Info_Module& info_drone = proto.info_drone();
        config.drone_module_data.docker_name = info_drone.docker_name();
        config.drone_module_data.docker_file = info_drone.docker_file();
        config.drone_module_data.module_ip = info_drone.module_ip();
        config.drone_module_data.ssh_ip = info_drone.ssh_ip();
        config.drone_module_data.port = info_drone.port();
        config.drone_module_data.user = info_drone.user();
        config.drone_module_data.key = info_drone.key();

        config.drone_sim = proto.drone_sim();

        return true;
    }

};