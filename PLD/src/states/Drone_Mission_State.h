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
    Drone_Mission_State(std::shared_ptr<State_Machine> state_machine_ptr);
    ~Drone_Mission_State();

    void start() override;
    void end() override;
    void handleMessage(const std::string &message) override;
    void set_data(Structs_PLD::Config_drone config);

private:
    Structs_PLD::Config_drone config_;
    int server_number_ = -1;
    std::shared_ptr<Docker_Manager> docker_manager_;
    boost::asio::steady_timer wait_timer_;
    boost::asio::steady_timer send_timer_;
    bool drone_module_running_;
    Struct_Drone::Status last_status_;
    int attemps_;
    size_t  drone_i_;
    size_t  coor_j_;
    bool state_closing_;

    void close_state();
    void continue_start_process(const boost::system::error_code& ec);
    void on_connect_drone();
    void on_error_drone(const boost::system::error_code& ec, const Type_Error &type_error);
    void on_message_drone(const std::string& msg);
    void send_message(const boost::system::error_code& ec);
};