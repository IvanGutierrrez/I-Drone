/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Algorithm_Manager.cpp               
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Algorithm_Manager.h"

Algorithm_Manager::Algorithm_Manager(std::shared_ptr<Communication_Manager> &comm_mng, 
                                     std::shared_ptr<Algorithm_Recorder> &rec_mng): comm_mng_ptr_(comm_mng),
                                                                                    recorder_ptr_(rec_mng)
{
}

Algorithm_Manager::~Algorithm_Manager()
{

}

void Algorithm_Manager::calculate()
{

}