/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : State_Machine.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <memory>
#include "./states/State.h"
#include "Communication_Manager.h"
#include "PLD_Recorder.h"

class State_Machine {
public:
    State_Machine(std::shared_ptr<Communication_Manager> comm_mng, std::shared_ptr<PLD_Recorder> recorder);
    void transitionTo(std::unique_ptr<State> next_state);
    void handleMessage(const std::string &message);
    std::shared_ptr<Communication_Manager> getCommunicationManager() const;
    std::shared_ptr<PLD_Recorder> getRecorder() const;
    boost::asio::io_context& get_io_context() const;
private:
    std::unique_ptr<State> actual_state_;
    std::shared_ptr<Communication_Manager> cmm_manager_;
    std::shared_ptr<PLD_Recorder> recorder_;

};