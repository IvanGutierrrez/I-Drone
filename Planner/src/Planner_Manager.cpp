/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_Manager.cpp               
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Planner_Manager.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <iomanip>
#include <functional>
#include "structs/Structs_Planner.h"
#include "common_libs/Logger.h"
#include "common_libs/Enc_Dec_PLD.h"

Planner_Manager::Planner_Manager(std::shared_ptr<Communication_Manager> comm_mng, 
                                     std::shared_ptr<Planner_Recorder> rec_mng,
                                     std::shared_ptr<Path_Cal> path_cal,
                                     std::shared_ptr<Signal_Cal> signal_cal,
                                     const Struct_Planner::Config_struct cnf): comm_mng_ptr_(std::move(comm_mng)),
                                                                            recorder_ptr_(std::move(rec_mng)),
                                                                            path_cal_ptr_(std::move(path_cal)),
                                                                            signal_cal_ptr_(std::move(signal_cal)),
                                                                            global_config_(cnf)
{
    comm_mng_ptr_->set_calculate_handler(std::bind(&Planner_Manager::calculate, this, std::placeholders::_1, std::placeholders::_2));
}

Planner_Manager::~Planner_Manager()
{
}

void Planner_Manager::calculate(const Struct_Planner::SignalServerConfig &config, Struct_Planner::DroneData drone_data)
{
    comm_mng_ptr_->set_status(Struct_Planner::Status::CALCULATING);
    auto points = signal_cal_ptr_->calculate_signal(global_config_,config);

    if (points.empty()) {
        comm_mng_ptr_->set_status(Struct_Planner::Status::ERROR);
        return;
    }

    Logger::log_message(Logger::Type::INFO, "Writting csv coverage map");
    recorder_ptr_->write_signal_output(points);

    std::stringstream log1;
    log1 << "Executing or tools Planner with " << points.size() << " points";
    Logger::log_message(Logger::Type::INFO, log1.str());

    std::vector<std::vector<Struct_Planner::Coordinate>> result;

    if (!path_cal_ptr_->calculate_path(drone_data,points,result,recorder_ptr_))
    {
        comm_mng_ptr_->set_status(Struct_Planner::Status::ERROR);
        return;
    }

    std::string msg;
    if (!Enc_Dec_PLD::encode_planner_response(result,msg))
    {
        Logger::log_message(Logger::Type::ERROR, "Error encoding Planner response");
        comm_mng_ptr_->set_status(Struct_Planner::Status::ERROR);
        return;
    }
    comm_mng_ptr_->deliver(msg);

    Logger::log_message(Logger::Type::INFO, "Planner_Manager task finish correctly");
    comm_mng_ptr_->set_status(Struct_Planner::Status::FINISH);
}