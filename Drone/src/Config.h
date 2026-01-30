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

inline bool get_common_config(Struct_Drone::Config_struct& cnf) {
    cnf.data_path = "/opt/I-Drone/data/";
    cnf.log_path = "/opt/I-Drone/logs";
    cnf.log_name = "drone";
    
    const char* num_env = std::getenv("NUM_DRONES");
    if (!num_env) {
        std::cerr << "ERROR: NUM_DRONES environment variable not set" << std::endl;
        return false;
    }
    
    try {
        cnf.num_drones = std::stoi(num_env);
        if (cnf.num_drones <= 0) {
            std::cerr << "ERROR: NUM_DRONES must be greater than 0" << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Invalid NUM_DRONES value: " << num_env << std::endl;
        return false;
    }
    
    return true;
}

inline bool get_drone_config(int drone_id, const Struct_Drone::Config_struct& common_config, Struct_Drone::Drone_Config& drone_cnf) {
    drone_cnf.drone_id = drone_id;
    
    // Read per-drone connection URL from environment
    std::string url_env_key = "DRONE_" + std::to_string(drone_id) + "_CONNECTION_URL";
    const char* url_env = std::getenv(url_env_key.c_str());
    if (!url_env) {
        std::cerr << "ERROR: " << url_env_key << " environment variable not set" << std::endl;
        return false;
    }
    drone_cnf.connection_url = url_env;
    
    // Read per-drone home position from environment
    std::string lat_env_key = "DRONE_" + std::to_string(drone_id) + "_HOME_LAT";
    std::string lon_env_key = "DRONE_" + std::to_string(drone_id) + "_HOME_LON";
    std::string alt_env_key = "DRONE_" + std::to_string(drone_id) + "_HOME_ALT";
    
    const char* lat_env = std::getenv(lat_env_key.c_str());
    const char* lon_env = std::getenv(lon_env_key.c_str());
    const char* alt_env = std::getenv(alt_env_key.c_str());
    
    if (!lat_env) {
        std::cerr << "ERROR: " << lat_env_key << " environment variable not set" << std::endl;
        return false;
    }
    if (!lon_env) {
        std::cerr << "ERROR: " << lon_env_key << " environment variable not set" << std::endl;
        return false;
    }
    if (!alt_env) {
        std::cerr << "ERROR: " << alt_env_key << " environment variable not set" << std::endl;
        return false;
    }
    
    try {
        drone_cnf.home_lat = std::stod(lat_env);
        drone_cnf.home_lon = std::stod(lon_env);
        drone_cnf.home_alt = std::stod(alt_env);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Invalid home position values for drone " << drone_id << std::endl;
        return false;
    }
    
    // Check for custom PX4 command
    std::string px4_cmd_env_key = "DRONE_" + std::to_string(drone_id) + "_PX4_COMMAND";
    const char* px4_cmd_env = std::getenv(px4_cmd_env_key.c_str());
    
    if (px4_cmd_env && std::string(px4_cmd_env) != "") {
        drone_cnf.command_px4 = std::string(px4_cmd_env);
    } else {
        // Build PX4 command using official multi-drone approach
        // Create separate working directory for each instance (like sitl_multiple_run.sh)
        std::string working_dir = common_config.data_path.string() + "px4_rootfs_" + std::to_string(drone_id);
        
        // Calculate spawn position
        double spawn_x = (drone_id % 2) * 2.0;
        double spawn_y = (drone_id / 2) * 2.0;
        
        drone_cnf.command_px4 = std::string("bash -c '") +
                                // Set Gazebo environment
                                "export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH} && " +
                                "export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH} && " +
                                // Wait for Gazebo
                                "timeout 30 bash -c \"until nc -z localhost 11345 2>/dev/null; do sleep 0.5; done\" && " +
                                // Staggered spawn
                                "sleep " + std::to_string(drone_id * 3) + " && " +
                                // Generate SDF using jinja (official method)
                                "/opt/I-Drone/generate_iris_sdf.sh " + std::to_string(drone_id) + 
                                " /tmp/iris_" + std::to_string(drone_id) + ".sdf && " +
                                // Spawn model
                                "gz model --spawn-file=/tmp/iris_" + std::to_string(drone_id) + ".sdf" +
                                " --model-name=iris_" + std::to_string(drone_id) +
                                " -x " + std::to_string(spawn_x) + " -y " + std::to_string(spawn_y) + " -z 0.83 && " +
                                "sleep 1 && " +
                                // Create working directory for this instance
                                "mkdir -p " + working_dir + " && " +
                                "cd " + working_dir + " && " +
                                // Run PX4 with instance-specific settings (no lockstep - official multi-drone approach)
                                "PX4_HOME_LAT=" + std::to_string(drone_cnf.home_lat) + 
                                " PX4_HOME_LON=" + std::to_string(drone_cnf.home_lon) + 
                                " PX4_HOME_ALT=" + std::to_string(drone_cnf.home_alt) + 
                                " PX4_SIM_MODEL=gazebo-classic_iris" +
                                " /root/tfg/PX4-Autopilot/build/px4_sitl_default/bin/px4" +
                                " -i " + std::to_string(drone_id) +
                                " -d /root/tfg/PX4-Autopilot/build/px4_sitl_default/etc" +
                                "' > " + common_config.data_path.string() + "px4_logs_" + std::to_string(drone_id) + ".txt 2>&1 &";
    }
    
    // Check if autostart should be disabled
    std::string autostart_env_key = "DRONE_" + std::to_string(drone_id) + "_AUTOSTART_PX4";
    const char* autostart_env = std::getenv(autostart_env_key.c_str());
    drone_cnf.autostart_px4 = true;
    if (autostart_env) {
        drone_cnf.autostart_px4 = (std::string(autostart_env) != "0" && std::string(autostart_env) != "false");
    }
    
    // Read takeoff altitude (optional, defaults to 10m)
    std::string takeoff_alt_env_key = "DRONE_" + std::to_string(drone_id) + "_TAKEOFF_ALTITUDE";
    const char* takeoff_alt_env = std::getenv(takeoff_alt_env_key.c_str());
    drone_cnf.takeoff_altitude_m = 10.0f; // Default
    if (takeoff_alt_env) {
        try {
            drone_cnf.takeoff_altitude_m = std::stof(takeoff_alt_env);
            if (drone_cnf.takeoff_altitude_m <= 0.0f) {
                std::cerr << "WARNING: Invalid takeoff altitude for drone " << drone_id << ", using default 10m" << std::endl;
                drone_cnf.takeoff_altitude_m = 10.0f;
            }
        } catch (const std::exception& e) {
            std::cerr << "WARNING: Could not parse takeoff altitude for drone " << drone_id << ", using default 10m" << std::endl;
            drone_cnf.takeoff_altitude_m = 10.0f;
        }
    }
    
    return true;
}

};