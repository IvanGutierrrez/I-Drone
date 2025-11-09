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
    
    bool encode_test(const MyMessage& msg, std::string &data) {
        return msg.SerializeToString(&data);
    }

    bool decode_test(const std::string& data, MyMessage &msg) {
        return msg.ParseFromString(data);
    }
};