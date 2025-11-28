/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Recorder.h                 
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>
#include <filesystem>
#include <fstream>

class Recorder {

public:
    Recorder() = default;
    Recorder(const std::filesystem::path &path, const std::string &filename, const std::string &extension);
    ~Recorder();
    
    bool write(const std::string text);
    void close();

private:
    std::filesystem::path filename_;
    std::ofstream file_;

    bool is_open();
    bool open_file();
};