/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Manager.cpp               
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Algorithm_Manager.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <iomanip>
#include <functional>
#include "structs/Structs_Algo.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_PLD.h"

Algorithm_Manager::Algorithm_Manager(std::shared_ptr<Communication_Manager> &comm_mng, 
                                     std::shared_ptr<Algorithm_Recorder> &rec_mng,
                                     std::shared_ptr<Path_Cal> &path_cal,
                                     std::shared_ptr<Signal_Cal> &signal_cal,
                                     const Struct_Algo::Config_struct cnf): comm_mng_ptr_(std::move(comm_mng)),
                                                                            recorder_ptr_(std::move(rec_mng)),
                                                                            path_cal_ptr_(std::move(path_cal)),
                                                                            signal_cal_ptr_(std::move(signal_cal)),
                                                                            global_config_(cnf)
{
    comm_mng_ptr_->set_calculate_handler(std::bind(&Algorithm_Manager::calculate, this, std::placeholders::_1, std::placeholders::_2));
}

Algorithm_Manager::~Algorithm_Manager()
{
}

void Algorithm_Manager::calculate(const Struct_Algo::SignalServerConfig &config, const Struct_Algo::DroneData &drone_data)
{
    comm_mng_ptr_->set_status(Struct_Algo::Status::CALCULATING);
    auto points = signal_cal_ptr_->calculate_signal(global_config_,config);

    if (points.empty()) {
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    // TODO Move to recorder class
    Logger::log_message(Logger::TYPE::INFO, "Writting csv coverage map");
    std::ofstream out(global_config_.executable_path + "/coverage_map.csv");
    out << "lat,lon,coverage\n";
    for (const auto& p : points)
        out << std::fixed << std::setprecision(6) << p.lat << "," << p.lon << ",1\n";
    out.close();

    std::stringstream log1;
    log1 << "Executing or tools algorithm with " << points.size() << " points";
    Logger::log_message(Logger::TYPE::INFO, log1.str());

    // TODO Calculate if there are targets which are not in points

    std::vector<std::vector<Struct_Algo::Coordinate>> result;

    if (!path_cal_ptr_->calculate_path(drone_data,points,result))
    {
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }

    std::string msg;
    if (!Enc_Dec_PLD::encode_algo_response(result,msg))
    {
        Logger::log_message(Logger::TYPE::ERROR, "Error encoding algorithm response");
        comm_mng_ptr_->set_status(Struct_Algo::Status::ERROR);
        return;
    }
    comm_mng_ptr_->deliver(msg);

    Logger::log_message(Logger::TYPE::INFO, "Algorithm_Manager task finish correctly");
    comm_mng_ptr_->set_status(Struct_Algo::Status::FINISH);
}