/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Drone_Mission_State.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <boost/asio.hpp>
#include "State.h"
#include "../Docker_Manager.h"
#include "structs/Structs_PLD.h"
#include "structs/Structs_Drone.h"
#include "common_libs/Server.h"

class Drone_Mission_State: public State {
public:
    explicit Drone_Mission_State(std::shared_ptr<State_Machine> state_machine_ptr);
    ~Drone_Mission_State() noexcept override;

    void start() override;
    void end() override;
    void set_data(const Structs_PLD::Config_drone &config);

private:
    Structs_PLD::Config_drone config_;
    int server_number_ = -1;
    std::shared_ptr<Docker_Manager> docker_manager_;
    boost::asio::steady_timer wait_timer_;
    boost::asio::steady_timer send_timer_;
    bool drone_module_running_ = false;
    Struct_Drone::Status last_status_ = Struct_Drone::Status::UNKNOWN;
    int attemps_ = 0;
    size_t  drone_i_ = 0;
    size_t  coor_j_ = 0;
    bool state_closing_ = false;

    const char* state_name() const override;
    void handle_finish_command() override;
    void close_state();
    void continue_start_process(const boost::system::error_code& ec);
    void on_connect_drone();
    void on_error_drone(const boost::system::error_code& ec, const Type_Error &type_error);
    void on_message_drone(const std::string& msg);
    void prepare_next_drone_message(Struct_Planner::Coordinate& coor, std::string& type);
    void send_message(const boost::system::error_code& ec);
};