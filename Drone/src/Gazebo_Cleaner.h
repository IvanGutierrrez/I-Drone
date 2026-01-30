/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Gazebo_Cleaner.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "Simulation_Cleaner.h"

// Concrete implementation for Gazebo + PX4 simulation cleanup
class Gazebo_Cleaner : public Simulation_Cleaner {
public:
    void cleanup() override;
};
