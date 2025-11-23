/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Path_Cal.h                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma once
#include "structs/Structs_Algo.h"
#include <iostream>
#include <memory>
#include <map>

class Path_Cal {

public:
    Path_Cal(const Struct_Algo::Config_struct &cnf);
    bool calculate_path(const Struct_Algo::DroneData &drone_data, 
                        const std::vector<Struct_Algo::Coordinate> &points, 
                        std::vector<std::vector<Struct_Algo::Coordinate>> &result);

private:
    Struct_Algo::Config_struct global_cnf_;

    double haversine_m(const Struct_Algo::Coordinate& a, const Struct_Algo::Coordinate& b);
    void build_knn_graph(const std::vector<Struct_Algo::Coordinate>& points,
                               int k_neighbors,
                               double max_neighbor_dist_m,
                               std::vector<std::vector<std::pair<int,double>>>& adj);
    std::vector<double> dijkstra(int src, const std::vector<std::vector<std::pair<int,double>>>& adj);
    void compute_target_distance_matrix(const std::vector<Struct_Algo::Coordinate>& all_points,
                                        const std::vector<std::vector<std::pair<int,double>>>& adj,
                                        const std::vector<Struct_Algo::Coordinate>& pos_targets,
                                        std::vector<std::vector<int64_t>>& dist_matrix);
    std::vector<std::vector<Struct_Algo::Coordinate>> solve_vrp(const std::vector<std::vector<int64_t>>& dist_matrix,
                                                  const std::vector<Struct_Algo::Coordinate>& pos_targets,
                                                  int num_drones);
};