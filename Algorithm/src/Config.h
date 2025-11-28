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

    cnf.data_path = "/opt/I-Drone/data/";
    cnf.log_path = "/opt/I-Drone/logs";
    cnf.log_name = "algorithm";
    cnf.signal_server_path = "/opt/signal_server/signal-server";
    cnf.executable_path = "/opt/I-Drone";
    cnf.threshold = -90.0;
    cnf.max_neighbor = 8;
    cnf.max_distance_for_neighbor = 100.0;
    cnf.max_ortools_time = 50;

    return cnf;
}

};