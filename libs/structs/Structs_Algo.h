/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Struct_algo.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>

namespace Struct_Algo {

enum class Status {
    OK,
    ERROR
};

inline std::string to_string(Status status) {
    switch (status) {
        case Status::OK:
            return "OK";
        case Status::ERROR:
            return "ERROR";
        default:
            return std::string();
    }
}


};
