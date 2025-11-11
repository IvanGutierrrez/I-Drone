/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Msg.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Enc_Dec_Msg.h"
#include <boost/asio/detail/socket_ops.hpp>
#include "structs/Structs_Algo.h"

namespace Enc_Dec {
    std::pair<Algo, std::unique_ptr<google::protobuf::Message>> decode_to_algo(const std::string& data)
    {
        Wrapper wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {Algo::UNKNOWN, nullptr};
        }

        if (wrapper.has_a()) {
            auto msg = std::make_unique<MyMessage>(wrapper.a());
            return {Algo::MyMessage, std::move(msg)};
        }

        if (wrapper.has_status()) {
            auto msg = std::make_unique<Status>(wrapper.status());
            return {Algo::Status, std::move(msg)};
        }

        return {Algo::UNKNOWN, nullptr};
    }
        
    bool encode_test(const MyMessage& msg, std::string &data) {
        return msg.SerializeToString(&data);
    }

    bool decode_test(const std::string& data, MyMessage &msg) {
        return msg.ParseFromString(data);
    }

    bool encode_status_algo(const Struct_Algo::Status &status, std::string &message)
    {
        Wrapper wrapper;

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