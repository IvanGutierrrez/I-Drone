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

};