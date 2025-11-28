/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Recorder.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Recorder.h"

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
    file_.flush(); // Force write to disk immediately
    return true;
}

void Recorder::close()
{
    if (is_open()) {
        file_.flush();
        file_.close();
    }
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