/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : PX4_Drone_Recorder.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "Drone_Recorder.h"
#include "common_libs/Recorder.h"
#include <string>
#include <filesystem>
#include <mutex>
#include <vector>

// Concrete implementation for PX4 drone recording with JSON output
// Writes data to /opt/I-Drone/data/drone_N/{telemetry,mission_events,command_logs}/*.json
class PX4_Drone_Recorder : public Drone_Recorder {
private:
    int drone_id_;
    
    std::vector<Telemetry_Record> telemetry_buffer_;
    std::vector<Mission_Event> events_buffer_;
    std::vector<Command_Log> commands_buffer_;
    
    Recorder recorder_telemetry_;
    Recorder recorder_events_;
    Recorder recorder_commands_;
    
    std::mutex telemetry_mutex_;
    std::mutex events_mutex_;
    std::mutex commands_mutex_;
    
    size_t buffer_size_;
    std::string session_timestamp_;
    
    std::string get_timestamp();
    std::string telemetry_to_json(const Telemetry_Record& record);
    std::string event_to_json(const Mission_Event& event);
    std::string command_to_json(const Command_Log& log);

public:
    PX4_Drone_Recorder(int drone_id, const std::string& base_data_path, size_t buffer_size = 100);
    ~PX4_Drone_Recorder() override;
    
    void log_telemetry(const Telemetry_Record& record) override;
    void log_mission_event(const Mission_Event& event) override;
    void log_command(const Command_Log& log) override;
    void flush() override;
};
