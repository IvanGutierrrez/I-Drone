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

namespace Config {

const std::filesystem::path log_path = "/home/ivan/repo/I-Drone/logs";
const std::string log_name = "algorithm";
const std::string signal_server_path = "/home/ivan/repo/signal_server_val/Signal-Server/signal_server";
const std::string executable_path = "/home/ivan/repo/I-Drone/I-Drone/build/Algorithm";
const double threshold = -150.0;

};