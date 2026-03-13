/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Off_State.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "State.h"
#include "common_libs/Enc_Dec_PLD.h"
#include "structs/Structs_PLD.h"

class Off_State: public State {
public:
    explicit Off_State(std::shared_ptr<State_Machine> state_machine_ptr);
    ~Off_State() override = default;

    void start() override;
    void end() override;
    void handleMessage(const std::string &message) override;

private:
    void handle_config_mission_message(const google::protobuf::Message* proto_msg, const std::string &raw_message);
    void handle_command_message(const google::protobuf::Message* proto_msg, const std::string &raw_message);
    void handle_unexpected_message(Enc_Dec_PLD::PLD type, const std::string &raw_message);
    Structs_PLD::Config_mission config_;

};