/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PX4_Drone_Recorder.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "PX4_Drone_Recorder.h"
#include "common_libs/Logger.h"
#include <chrono>
#include <iomanip>
#include <sstream>

PX4_Drone_Recorder::PX4_Drone_Recorder(int drone_id, const std::string& base_data_path, size_t buffer_size, const std::string& session_timestamp)
    : drone_id_(drone_id), 
      buffer_size_(buffer_size) {
    
    session_timestamp_ = session_timestamp.empty() ? get_timestamp() : session_timestamp;
    
    // Create session folder: base_path/drone_[timestamp]/
    std::filesystem::path session_folder = std::filesystem::path(base_data_path) / ("drone_" + session_timestamp_);
    
    // Initialize recorders with session folder
    recorder_telemetry_ = std::make_unique<Recorder>(session_folder / "telemetry", "telemetry_" + session_timestamp_, "json");
    recorder_events_ = std::make_unique<Recorder>(session_folder / "mission_events", "events_" + session_timestamp_, "json");
    recorder_commands_ = std::make_unique<Recorder>(session_folder / "command_logs", "commands_" + session_timestamp_, "json");
    
    Logger::log_message(Logger::Type::INFO, "PX4_Drone_Recorder initialized for drone " + 
                       std::to_string(drone_id_) + " at " + session_folder.string());
}

PX4_Drone_Recorder::~PX4_Drone_Recorder() {
    flush();
    if (recorder_telemetry_) recorder_telemetry_->close();
    if (recorder_events_) recorder_events_->close();
    if (recorder_commands_) recorder_commands_->close();
}

std::string PX4_Drone_Recorder::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    ss << '_' << std::setfill('0') << std::setw(6) << us.count();
    
    return ss.str();
}

std::string PX4_Drone_Recorder::telemetry_to_json(const Telemetry_Record& record) {
    std::stringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{"
         << "\"timestamp\":" << record.timestamp << ","
         << "\"drone_id\":" << record.drone_id << ","
         << "\"latitude_deg\":" << record.latitude_deg << ","
         << "\"longitude_deg\":" << record.longitude_deg << ","
         << "\"absolute_altitude_m\":" << record.absolute_altitude_m << ","
         << "\"relative_altitude_m\":" << record.relative_altitude_m << ","
         << "\"velocity_north_m_s\":" << record.velocity_north_m_s << ","
         << "\"velocity_east_m_s\":" << record.velocity_east_m_s << ","
         << "\"velocity_down_m_s\":" << record.velocity_down_m_s << ","
         << "\"roll_deg\":" << record.roll_deg << ","
         << "\"pitch_deg\":" << record.pitch_deg << ","
         << "\"yaw_deg\":" << record.yaw_deg << ","
         << "\"armed\":" << (record.armed ? "true" : "false") << ","
         << "\"in_air\":" << (record.in_air ? "true" : "false") << ","
         << "\"flight_mode\":\"" << record.flight_mode << "\","
         << "\"battery_voltage_v\":" << record.battery_voltage_v << ","
         << "\"battery_remaining_percent\":" << record.battery_remaining_percent
         << "}";
    return json.str();
}

std::string PX4_Drone_Recorder::event_to_json(const Mission_Event& event) {
    std::stringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{"
         << "\"timestamp\":" << event.timestamp << ","
         << "\"drone_id\":" << event.drone_id << ","
         << "\"event_type\":\"" << event.event_type << "\","
         << "\"waypoint_index\":" << event.waypoint_index << ","
         << "\"latitude_deg\":" << event.latitude_deg << ","
         << "\"longitude_deg\":" << event.longitude_deg
         << "}";
    return json.str();
}

std::string PX4_Drone_Recorder::command_to_json(const Command_Log& log) {
    std::stringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{"
         << "\"timestamp\":" << log.timestamp << ","
         << "\"drone_id\":" << log.drone_id << ","
         << "\"command_type\":\"" << log.command_type << "\","
         << "\"result\":\"" << log.result << "\","
         << "\"details\":\"" << log.details << "\""
         << "}";
    return json.str();
}

void PX4_Drone_Recorder::log_telemetry(const Telemetry_Record& record) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetry_buffer_.push_back(record);
    
    if (telemetry_buffer_.size() >= buffer_size_) {
        std::stringstream json_array;
        json_array << "[\n";
        for (size_t i = 0; i < telemetry_buffer_.size(); ++i) {
            json_array << "  " << telemetry_to_json(telemetry_buffer_[i]);
            if (i < telemetry_buffer_.size() - 1) json_array << ",";
            json_array << "\n";
        }
        json_array << "]\n";
        
        if (recorder_telemetry_) recorder_telemetry_->write(json_array.str());
        telemetry_buffer_.clear();
    }
}

void PX4_Drone_Recorder::log_mission_event(const Mission_Event& event) {
    std::lock_guard<std::mutex> lock(events_mutex_);
    events_buffer_.push_back(event);
    
    if (events_buffer_.size() >= buffer_size_) {
        std::stringstream json_array;
        json_array << "[\n";
        for (size_t i = 0; i < events_buffer_.size(); ++i) {
            json_array << "  " << event_to_json(events_buffer_[i]);
            if (i < events_buffer_.size() - 1) json_array << ",";
            json_array << "\n";
        }
        json_array << "]\n";
        
        if (recorder_events_) recorder_events_->write(json_array.str());
        events_buffer_.clear();
    }
}

void PX4_Drone_Recorder::log_command(const Command_Log& log) {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    commands_buffer_.push_back(log);
    
    if (commands_buffer_.size() >= buffer_size_) {
        std::stringstream json_array;
        json_array << "[\n";
        for (size_t i = 0; i < commands_buffer_.size(); ++i) {
            json_array << "  " << command_to_json(commands_buffer_[i]);
            if (i < commands_buffer_.size() - 1) json_array << ",";
            json_array << "\n";
        }
        json_array << "]\n";
        
        if (recorder_commands_) recorder_commands_->write(json_array.str());
        commands_buffer_.clear();
    }
}

void PX4_Drone_Recorder::flush() {
    // Flush remaining telemetry
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (!telemetry_buffer_.empty()) {
            std::stringstream json_array;
            json_array << "[\n";
            for (size_t i = 0; i < telemetry_buffer_.size(); ++i) {
                json_array << "  " << telemetry_to_json(telemetry_buffer_[i]);
                if (i < telemetry_buffer_.size() - 1) json_array << ",";
                json_array << "\n";
            }
            json_array << "]\n";
            
            if (recorder_telemetry_) recorder_telemetry_->write(json_array.str());
            telemetry_buffer_.clear();
        }
    }
    
    // Flush remaining events
    {
        std::lock_guard<std::mutex> lock(events_mutex_);
        if (!events_buffer_.empty()) {
            std::stringstream json_array;
            json_array << "[\n";
            for (size_t i = 0; i < events_buffer_.size(); ++i) {
                json_array << "  " << event_to_json(events_buffer_[i]);
                if (i < events_buffer_.size() - 1) json_array << ",";
                json_array << "\n";
            }
            json_array << "]\n";
            
            if (recorder_events_) recorder_events_->write(json_array.str());
            events_buffer_.clear();
        }
    }
    
    // Flush remaining commands
    {
        std::lock_guard<std::mutex> lock(commands_mutex_);
        if (!commands_buffer_.empty()) {
            std::stringstream json_array;
            json_array << "[\n";
            for (size_t i = 0; i < commands_buffer_.size(); ++i) {
                json_array << "  " << command_to_json(commands_buffer_[i]);
                if (i < commands_buffer_.size() - 1) json_array << ",";
                json_array << "\n";
            }
            json_array << "]\n";
            
            if (recorder_commands_) recorder_commands_->write(json_array.str());
            commands_buffer_.clear();
        }
    }
    
    Logger::log_message(Logger::Type::INFO, "Drone " + std::to_string(drone_id_) + " recorder flushed");
}
