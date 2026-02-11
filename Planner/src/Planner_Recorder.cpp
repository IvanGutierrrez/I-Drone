/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_Recorder.cpp                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Planner_Recorder.h"
#include "common_libs/Logger.h"
#include <chrono>
#include <iomanip>

constexpr const char message_received_file_name[] = "message_received";
constexpr const char message_received_file_extension[] = "txt";
constexpr const char signal_server_output_file_name[] = "output_signal";
constexpr const char signal_server_output_file_extension[] = "csv";
constexpr const char ortools_output_file_name[] = "output_ortools";
constexpr const char ortools_output_file_extension[] = "txt";

static std::string get_session_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    ss << '_' << std::setfill('0') << std::setw(6) << us.count();
    
    return ss.str();
}

Planner_Recorder::Planner_Recorder(const std::filesystem::path &path)
{
    std::string session_folder = "planner_" + get_session_timestamp();
    std::filesystem::path session_path = path / session_folder;
    
    recorder_msg = std::make_unique<Recorder>(session_path, message_received_file_name, message_received_file_extension);
    recorder_sgn = std::make_unique<Recorder>(session_path, signal_server_output_file_name, signal_server_output_file_extension);
    recorder_or = std::make_unique<Recorder>(session_path, ortools_output_file_name, ortools_output_file_extension);
    
    Logger::log_message(Logger::Type::INFO, "Planner_Recorder initialized at " + session_path.string());
}

bool Planner_Recorder::write_signal_output(const std::vector<Struct_Planner::Coordinate> &points)
{
    if (!recorder_sgn) return false;
    
    std::stringstream data;
    data << "lat,lon,coverage\n";
    for (const auto& p : points)
        data << std::fixed << std::setprecision(6) << p.lat << "," << p.lon << ",1\n";
    return recorder_sgn->write(data.str());
}

bool Planner_Recorder::write_message_received(const Struct_Planner::SignalServerConfig &sng_data, const Struct_Planner::DroneData &drone_data)
{
    if (!recorder_msg) return false;
    
    std::stringstream data;
    data << sng_data << "\n" << drone_data;
    return recorder_msg->write(data.str());
}

bool Planner_Recorder::write_or_output(const std::string &data)
{
    if (!recorder_or) return false;
    return recorder_or->write(data);
}

void Planner_Recorder::close_all()
{
    if (recorder_msg) recorder_msg->close();
    if (recorder_sgn) recorder_sgn->close();
    if (recorder_or) recorder_or->close();
    Logger::log_message(Logger::Type::INFO, "All recorder files closed successfully");
}