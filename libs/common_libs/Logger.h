/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Logger.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "Recorder.h"

namespace Logger{

enum class TYPE{
    INFO,
    WARNING,
    ERROR
};

std::string to_string(const TYPE &t);
bool initialize(const std::filesystem::path &path, 
                       const std::string &name);
void log_message(const TYPE &t, const std::string &log);
std::string getTimeFormatted();
std::string getCurrentTimestamp();
};