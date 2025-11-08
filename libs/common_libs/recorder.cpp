/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : recorder.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "recorder.h"

Recorder::Recorder(const std::filesystem::path &path, 
                   const std::string &filename, 
                   const std::string &extension): filename_(path / (filename + "." + extension))
{}

Recorder::~Recorder()
{
    if (is_open())
        file_.close();
}

bool Recorder::write(const std::string text)
{

    if (!is_open()) {
        if (!open_file()){
            return false;
        }
    }
    file_ << text;
    return true;
}

bool Recorder::open_file()
{
    try {
        std::filesystem::create_directories(filename_.parent_path());
        file_.open(filename_, std::ios::out | std::ios::app);

        return file_.is_open();
    } catch (const std::filesystem::filesystem_error& e) {
        return false;
    }
}

bool Recorder::is_open()
{
    return file_.is_open();
}