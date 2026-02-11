/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PLD_Recorder.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <iostream>
#include <memory>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "common_libs/Recorder.h"

class PLD_Recorder {

public:
    PLD_Recorder(const std::filesystem::path &path);

    bool write_message_received(const std::string &source, const std::string &message_type, const std::string &decoded_content);
    bool write_message_sent(const std::string &destination, const std::string &message_type, const std::string &content);
    bool write_raw_message(const std::string &source, const std::string &raw_data);
    bool write_state_transition(const std::string &from_state, const std::string &to_state);
    bool write_error(const std::string &error_message);
    void close();
    void start_new_session();

private:
    std::filesystem::path base_path_;
    std::unique_ptr<Recorder> recorder_;
    
    std::string get_timestamp();
    std::string get_filename_timestamp();
    bool write_json_entry(const std::string &event_type, const std::string &data);
    void create_new_recorder();
};
