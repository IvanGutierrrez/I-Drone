/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Config.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include <filesystem>
#include <cstdlib>
#include <string>
#include "structs/Structs_Drone.h"

namespace Config {

inline Struct_Drone::Config_struct get_config() {
    Struct_Drone::Config_struct cnf;

    cnf.data_path = "/opt/I-Drone/data/";
    cnf.log_path = "/opt/I-Drone/logs";
    cnf.log_name = "drone";
    
    // Read home position from environment variables (with defaults)
    const char* lat_env = std::getenv("PX4_HOME_LAT");
    const char* lon_env = std::getenv("PX4_HOME_LON");
    const char* alt_env = std::getenv("PX4_HOME_ALT");
    
    std::string lat = lat_env ? lat_env : "0";
    std::string lon = lon_env ? lon_env : "0";
    std::string alt = alt_env ? alt_env : "0";
    
    cnf.command_px4 = "PX4_HOME_LAT=" + lat + " PX4_HOME_LON=" + lon + " PX4_HOME_ALT=" + alt + 
                      " make -C /root/tfg/PX4-Autopilot px4_sitl gazebo-classic_iris > " + 
                      cnf.data_path.string() + "px4_logs.txt 2>&1 &"; // PX4_INSTANCE=0 

    return cnf;
}

};