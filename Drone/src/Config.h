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
#include "structs/Structs_Drone.h"

namespace Config {

inline Struct_Drone::Config_struct get_config() {
    Struct_Drone::Config_struct cnf;

    cnf.data_path = "/opt/I-Drone/data/";
    cnf.log_path = "/home/ivan/repo/I-Drone/logs";//"/opt/I-Drone/logs";
    cnf.log_name = "drone";

    return cnf;
}

};