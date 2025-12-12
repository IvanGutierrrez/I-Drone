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

constexpr const char message_received_file_name[] = "message_received";
constexpr const char message_received_file_extension[] = "txt";
constexpr const char signal_server_output_file_name[] = "output_signal";
constexpr const char signal_server_output_file_extension[] = "csv";
constexpr const char ortools_output_file_name[] = "output_ortools";
constexpr const char ortools_output_file_extension[] = "txt";

Planner_Recorder::Planner_Recorder(const std::filesystem::path &path): recorder_msg(path,message_received_file_name,message_received_file_extension),
                                                                           recorder_sgn(path,signal_server_output_file_name,signal_server_output_file_extension),
                                                                           recorder_or(path,ortools_output_file_name,ortools_output_file_extension)
{
}

bool Planner_Recorder::write_signal_output(const std::vector<Struct_Planner::Coordinate> &points)
{
    std::stringstream data;
    data << "lat,lon,coverage\n";
    for (const auto& p : points)
        data << std::fixed << std::setprecision(6) << p.lat << "," << p.lon << ",1\n";
    return recorder_sgn.write(data.str());
}

bool Planner_Recorder::write_message_received(const Struct_Planner::SignalServerConfig &sng_data, const Struct_Planner::DroneData &drone_data)
{
    std::stringstream data;
    data << sng_data << "\n" << drone_data;
    return recorder_msg.write(data.str());
}

bool Planner_Recorder::write_or_output(const std::string &data)
{
    return recorder_or.write(data);
}

void Planner_Recorder::close_all()
{
    recorder_msg.close();
    recorder_sgn.close();
    recorder_or.close();
    Logger::log_message(Logger::Type::INFO, "All recorder files closed successfully");
}