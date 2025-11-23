/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Config.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include <iostream>
#include <filesystem>
#include "structs/Structs_Algo.h"

namespace Config {

inline Struct_Algo::Config_struct get_config() {
    Struct_Algo::Config_struct cnf;

    cnf.log_path = "/home/ivan/repo/I-Drone/logs";
    cnf.log_name = "algorithm";
    cnf.signal_server_path = "/home/ivan/repo/signal_server_val/Signal-Server/signal_server";
    cnf.executable_path = "/home/ivan/repo/I-Drone/I-Drone/build/Algorithm";
    cnf.threshold = -90.0;

    return cnf;
}

};