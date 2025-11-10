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

        return {Algo::UNKNOWN, nullptr};
    }
        
    bool encode_test(const MyMessage& msg, std::string &data) {
        return msg.SerializeToString(&data);
    }

    bool decode_test(const std::string& data, MyMessage &msg) {
        return msg.ParseFromString(data);
    }
};