/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Multi_Drone_Manager.h                   
 *  Author   : Iván Gutiérrez                           
 *  License  : GNU General Public License v3.0          
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "Engine.h"

class Multi_Drone_Manager {
public:
    struct Handlers {
        f_handler_normal all_missions_complete = nullptr;
        std::function<void(int drone_id)> error = nullptr;
        f_handler_normal missions_ready = nullptr;
    };

    explicit Multi_Drone_Manager(std::vector<std::shared_ptr<Engine>> engines);
    ~Multi_Drone_Manager();

    void set_handlers(Handlers handlers);
    void start_all();
    void dispatch_command(const std::string &message);
    void flush_all_recorders();

private:
    void on_drone_complete();
    void on_drone_error(int drone_id);
    void ensure_start_signal();

    std::vector<std::shared_ptr<Engine>> drones_;
    std::vector<std::thread> threads_;
    Handlers handlers_{};

    std::shared_ptr<std::promise<void>> start_promise_;
    std::shared_future<void> start_future_;
    std::once_flag start_once_flag_;
    std::once_flag error_once_flag_;

    std::atomic<size_t> completed_{0};
    std::atomic<size_t> next_drone_{0};
    std::mutex handlers_mutex_;
};
