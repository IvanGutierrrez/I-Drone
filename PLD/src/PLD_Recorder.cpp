/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PLD_Recorder.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "PLD_Recorder.h"
#include "common_libs/Logger.h"

constexpr const char messages_file_name[] = "pld_messages";
constexpr const char messages_file_extension[] = "json";

PLD_Recorder::PLD_Recorder(const std::filesystem::path &path): base_path_(path)
{
    create_new_recorder();
}

void PLD_Recorder::create_new_recorder()
{
    std::string filename_with_timestamp = std::string(messages_file_name) + "_" + get_filename_timestamp();
    recorder_ = std::make_unique<Recorder>(base_path_, filename_with_timestamp, messages_file_extension);
    // Write opening bracket for JSON array
    recorder_->write("[\n");
}

std::string PLD_Recorder::get_timestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

std::string PLD_Recorder::get_filename_timestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    ss << '_' << std::setfill('0') << std::setw(6) << us.count();
    
    return ss.str();
}

bool PLD_Recorder::write_json_entry(const std::string &event_type, const std::string &data)
{
    if (!recorder_) return false;
    
    std::stringstream json;
    json << "  {\n";
    json << "    \"timestamp\": \"" << get_timestamp() << "\",\n";
    json << "    \"event_type\": \"" << event_type << "\",\n";
    json << "    \"data\": " << data << "\n";
    json << "  },\n";
    
    return recorder_->write(json.str());
}

bool PLD_Recorder::write_message_received(const std::string &source, const std::string &message_type, const std::string &decoded_content)
{
    std::stringstream data;
    data << "{\n";
    data << "      \"direction\": \"received\",\n";
    data << "      \"source\": \"" << source << "\",\n";
    data << "      \"message_type\": \"" << message_type << "\",\n";
    data << "      \"content\": \"" << decoded_content << "\"\n";
    data << "    }";
    
    return write_json_entry("message", data.str());
}

bool PLD_Recorder::write_message_sent(const std::string &destination, const std::string &message_type, const std::string &content)
{
    std::stringstream data;
    data << "{\n";
    data << "      \"direction\": \"sent\",\n";
    data << "      \"destination\": \"" << destination << "\",\n";
    data << "      \"message_type\": \"" << message_type << "\",\n";
    data << "      \"content\": \"" << content << "\"\n";
    data << "    }";
    
    return write_json_entry("message", data.str());
}

bool PLD_Recorder::write_raw_message(const std::string &source, const std::string &raw_data)
{
    std::stringstream data;
    data << "{\n";
    data << "      \"source\": \"" << source << "\",\n";
    data << "      \"raw_data\": \"" << raw_data << "\"\n";
    data << "    }";
    
    return write_json_entry("raw_message", data.str());
}

bool PLD_Recorder::write_state_transition(const std::string &from_state, const std::string &to_state)
{
    std::stringstream data;
    data << "{\n";
    data << "      \"from\": \"" << from_state << "\",\n";
    data << "      \"to\": \"" << to_state << "\"\n";
    data << "    }";
    
    return write_json_entry("state_transition", data.str());
}

bool PLD_Recorder::write_error(const std::string &error_message)
{
    std::stringstream data;
    data << "{\n";
    data << "      \"error\": \"" << error_message << "\"\n";
    data << "    }";
    
    return write_json_entry("error", data.str());
}

void PLD_Recorder::close()
{
    if (!recorder_) return;
    
    // Write closing bracket for JSON array
    recorder_->write("  {}\n]\n");
    recorder_->close();
    Logger::log_message(Logger::Type::INFO, "PLD Recorder closed successfully");
}

void PLD_Recorder::start_new_session()
{
    if (recorder_) {
        close();
    }
    create_new_recorder();
    Logger::log_message(Logger::Type::INFO, "Started new PLD Recorder session");
}
