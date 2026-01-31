/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Multi_Drone_Manager.cpp                   
 *  Author   : Iván Gutiérrez                           
 *  License  : GNU General Public License v3.0          
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Multi_Drone_Manager.h"

#include "common_libs/Enc_Dec_Drone.h"
#include "common_libs/Logger.h"
#include <functional>

Multi_Drone_Manager::Multi_Drone_Manager(std::vector<std::shared_ptr<Engine>> engines)
{
    start_promise_ = std::make_shared<std::promise<void>>();
    start_future_ = start_promise_->get_future().share();

    drones_ = std::move(engines);
    for (auto &drone : drones_) {
        drone->set_start_signal(start_future_);
    }
}

Multi_Drone_Manager::~Multi_Drone_Manager()
{
    ensure_start_signal();
    for (auto &t : threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
}

void Multi_Drone_Manager::set_handlers(Handlers handlers)
{
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    handlers_ = std::move(handlers);

    for (size_t i = 0; i < drones_.size(); ++i) {
        Engine::Handlers drone_handlers;
        drone_handlers.mission_complete = std::bind(&Multi_Drone_Manager::on_drone_complete, this);
        drone_handlers.error = [this, i](int drone_id) { this->on_drone_error(drone_id); };
        drones_[i]->set_handler(drone_handlers);
    }
}

void Multi_Drone_Manager::start_all()
{
    for (auto &drone : drones_) {
        threads_.emplace_back(&Engine::start_engine, drone.get());
    }
}

void Multi_Drone_Manager::dispatch_command(const std::string &message)
{
    auto decoded = Enc_Dec_Drone::decode_to_drone(message);
    if (decoded.first == Enc_Dec_Drone::Drone::COMMAND) {
        if (auto command = dynamic_cast<DroneCommandString*>(decoded.second.get())) {
            if (command->type_command() == "START_ALL") {
                for (auto &drone : drones_) {
                    drone->mark_commands_ready();
                }
                
                // Release start signal to allow drones to takeoff
                ensure_start_signal();
                
                // Notify that missions are ready to execute
                {
                    std::lock_guard<std::mutex> lock(handlers_mutex_);
                    if (handlers_.missions_ready) {
                        handlers_.missions_ready();
                    }
                }
            } else {
                // Group commands by START-FINISH blocks
                // START begins a mission for current drone, FINISH completes it
                std::string cmd_type = command->type_command();
                
                if (cmd_type == "START") {
                    current_drone_index_ = (current_drone_index_ + 1) % drones_.size();
                    Logger::log_message(Logger::Type::INFO, 
                        "Starting mission block for Drone " + std::to_string(current_drone_index_));
                }
                
                // Send command to current drone
                drones_[current_drone_index_]->send_command(message);
                
                if (cmd_type == "FINISH") {
                    Logger::log_message(Logger::Type::INFO, 
                        "Finished mission block for Drone " + std::to_string(current_drone_index_));
                }
            }
        }
    } else if (decoded.first == Enc_Dec_Drone::Drone::ERROR) {
        Logger::log_message(Logger::Type::ERROR, "Error decoding message in Multi_Drone_Manager");
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        if (handlers_.error) {
            handlers_.error(-1); // -1 indicates error not from specific drone
        }
        return;
    } else if (decoded.first == Enc_Dec_Drone::Drone::UNKNOWN) {
        Logger::log_message(Logger::Type::WARNING, "Unknown message type in Multi_Drone_Manager");
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        if (handlers_.error) {
            handlers_.error(-1);
        }
    }
}

void Multi_Drone_Manager::flush_all_recorders()
{
    for (auto& drone : drones_) {
        drone->flush_recorder();
    }
}

void Multi_Drone_Manager::on_drone_complete()
{
    const auto done = completed_.fetch_add(1) + 1;
    if (done == drones_.size()) {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        if (handlers_.all_missions_complete) {
            handlers_.all_missions_complete();
        }
    }
}

void Multi_Drone_Manager::on_drone_error(int drone_id)
{
    std::call_once(error_once_flag_, [this, drone_id]() {
        Logger::log_message(Logger::Type::ERROR, "Drone " + std::to_string(drone_id) + " reported an error");
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        if (handlers_.error) {
            handlers_.error(drone_id);
        }
    });
}

void Multi_Drone_Manager::ensure_start_signal()
{
    std::call_once(start_once_flag_, [this]() {
        Logger::log_message(Logger::Type::INFO, "Releasing synchronized start signal for all drones");
        start_promise_->set_value();
    });
}
