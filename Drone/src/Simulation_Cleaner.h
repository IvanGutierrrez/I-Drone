/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Simulation_Cleaner.h                   
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once

// Interface for simulation cleanup
class Simulation_Cleaner {
public:
    virtual ~Simulation_Cleaner() = default;
    virtual void cleanup() = 0;
};
