/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Planner_State.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <boost/asio.hpp>
#include "State.h"
#include "structs/Structs_PLD.h"
#include "common_libs/Docker_Manager.h"
#include "common_libs/Server.h"

class Planner_State: public State {
public:
    Planner_State(std::shared_ptr<State_Machine> state_machine_ptr);
    ~Planner_State();

    void start() override;
    void end() override;
    void handleMessage(const std::string &message) override;
    void set_data(Structs_PLD::Config_mission config);

private:
    Structs_PLD::Config_mission config_;
    int server_number_ = -1;
    std::shared_ptr<Docker_Manager> docker_manager_;
    boost::asio::steady_timer wait_timer_;
    bool planner_running_;
    bool response_message_received_;
    Struct_Planner::Status last_status_;
    int attemps_;

    void continue_start_process(const boost::system::error_code& ec);
    void on_connect_planner();
    void on_error_planner(const boost::system::error_code& ec, const Type_Error &type_error);
    void on_message_planner(const std::string& msg);
};