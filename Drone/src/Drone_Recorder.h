/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Drone_Recorder.h                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <string>

// Interface for drone data recording
class Drone_Recorder {
public:
    struct Telemetry_Record {
        double timestamp;
        int drone_id;
        double latitude_deg;
        double longitude_deg;
        double absolute_altitude_m;
        double relative_altitude_m;
        float velocity_north_m_s;
        float velocity_east_m_s;
        float velocity_down_m_s;
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        bool armed;
        bool in_air;
        std::string flight_mode;
        float battery_voltage_v;
        float battery_remaining_percent;
    };

    struct Mission_Event {
        double timestamp;
        int drone_id;
        std::string event_type;
        int waypoint_index;
        double latitude_deg;
        double longitude_deg;
    };

    struct Command_Log {
        double timestamp;
        int drone_id;
        std::string command_type;
        std::string result;
        std::string details;
    };

    virtual ~Drone_Recorder() = default;
    
    virtual void log_telemetry(const Telemetry_Record& record) = 0;
    virtual void log_mission_event(const Mission_Event& event) = 0;
    virtual void log_command(const Command_Log& log) = 0;
    virtual void flush() = 0;
};
