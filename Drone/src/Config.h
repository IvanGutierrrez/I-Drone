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
        // Build PX4 command: spawn model + run PX4 directly (avoid sitl_run.sh)
        // Calculate spawn position based on drone_id
        double spawn_x = (drone_id % 2) * 2.0;  // 0, 2, 0, 2...
        double spawn_y = (drone_id / 2) * 2.0;  // 0, 0, 2, 2...
        
        drone_cnf.command_px4 = std::string("bash -c '") +
                                // Set environment for Gazebo interaction
                                "export GAZEBO_MODEL_PATH=/root/tfg/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH} && " +
                                "export GAZEBO_PLUGIN_PATH=/root/tfg/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH} && " +
                                // Wait for Gazebo to be ready (poll port 11345)
                                "timeout 30 bash -c \"until nc -z localhost 11345 2>/dev/null; do sleep 0.5; done\" && " +
                                // Staggered delay to ensure sequential spawning
                                "sleep " + std::to_string(drone_id * 3) + " && " +
                                // Generate custom SDF with unique TCP port
                                "/opt/I-Drone/generate_iris_sdf.sh " + std::to_string(drone_id) + 
                                " /tmp/iris_" + std::to_string(drone_id) + ".sdf && " +
                                // Spawn iris model with unique name and custom SDF
                                "gz model --spawn-file=/tmp/iris_" + std::to_string(drone_id) + ".sdf" +
                                " --model-name=iris_" + std::to_string(drone_id) +
                                " -x " + std::to_string(spawn_x) + " -y " + std::to_string(spawn_y) + " -z 0.1 && " +
                                "sleep 1 && " +
                                // Run PX4
                                "cd /root/tfg/PX4-Autopilot/build/px4_sitl_default && " +
                                "PX4_HOME_LAT=" + std::to_string(drone_cnf.home_lat) + 
                                " PX4_HOME_LON=" + std::to_string(drone_cnf.home_lon) + 
                                " PX4_HOME_ALT=" + std::to_string(drone_cnf.home_alt) + 
                                " PX4_SYS_AUTOSTART=4001" +
                                " PX4_INSTANCE=" + std::to_string(drone_id) +
                                " PX4_SIMULATOR=gazebo-classic" +
                                " PX4_SIM_MODEL=iris" +
                                " PX4_GZ_MODEL_NAME=iris_" + std::to_string(drone_id) +
                                // Only drone 0 uses lockstep (drives Gazebo time), others use real-time
                                (drone_id == 0 ? "" : " PX4_SIM_SPEED_FACTOR=1") +
                                " bin/px4 -i " + std::to_string(drone_id) + " -d -s etc/init.d-posix/rcS" +
                                "' > " + common_config.data_path.string() + "px4_logs_" + std::to_string(drone_id) + ".txt 2>&1 &";
    }
    
    // Check if autostart should be disabled
    std::string autostart_env_key = "DRONE_" + std::to_string(drone_id) + "_AUTOSTART_PX4";
    const char* autostart_env = std::getenv(autostart_env_key.c_str());
    drone_cnf.autostart_px4 = true;
    if (autostart_env) {
        drone_cnf.autostart_px4 = (std::string(autostart_env) != "0" && std::string(autostart_env) != "false");
    }
    
    return true;
}

};