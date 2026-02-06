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
#include "structs/Structs_PLD.h"

namespace Config {

inline Structs_PLD::Config_struct get_config() {
    Structs_PLD::Config_struct cnf;

    cnf.data_path = "/opt/I-Drone/data/";
    cnf.log_path = "/opt/I-Drone/logs";
    cnf.log_name = "pld";

    return cnf;
}

};