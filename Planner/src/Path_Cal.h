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
#include "structs/Structs_Planner.h"
#include "Planner_Recorder.h"
#include <iostream>
#include <memory>
#include <map>

class Path_Cal {

public:
    Path_Cal(const Struct_Planner::Config_struct &cnf);
    bool calculate_path(Struct_Planner::DroneData &drone_data, 
                        std::vector<Struct_Planner::Coordinate> &points, 
                        std::vector<std::vector<Struct_Planner::Coordinate>> &result,
                        const std::shared_ptr<Planner_Recorder> &rec_mng);

private:
    Struct_Planner::Config_struct global_cnf_;

    std::vector<size_t> findNearestPoints(const std::vector<Struct_Planner::Coordinate> &points_cp, const Struct_Planner::DroneData &drone_data);
    bool check_targets_signal(Struct_Planner::DroneData &drone_data, const std::vector<Struct_Planner::Coordinate> &points_cp);
    double haversine_m(const Struct_Planner::Coordinate& a, const Struct_Planner::Coordinate& b);
    void build_knn_graph(const std::vector<Struct_Planner::Coordinate>& points,
                               int k_neighbors,
                               double max_neighbor_dist_m,
                               std::vector<std::vector<std::pair<int,double>>>& adj);
    std::vector<double> dijkstra(int src, const std::vector<std::vector<std::pair<int,double>>>& adj);
    std::vector<int> dijkstra_path(int src, int tgt, const std::vector<std::vector<std::pair<int,double>>>& adj);
    void compute_target_distance_matrix(const std::vector<Struct_Planner::Coordinate>& all_points,
                                        const std::vector<std::vector<std::pair<int,double>>>& adj,
                                        const std::vector<Struct_Planner::Coordinate>& pos_targets,
                                        std::vector<std::vector<int64_t>>& dist_matrix);
    std::vector<std::vector<Struct_Planner::Coordinate>> solve_vrp(const std::vector<std::vector<int64_t>>& dist_matrix,
                                                                const std::vector<Struct_Planner::Coordinate>& pos_targets,
                                                                int num_drones,
                                                                const std::vector<Struct_Planner::Coordinate> &points_cp,
                                                                const std::vector<std::vector<std::pair<int,double>>>& adj,
                                                                const std::shared_ptr<Planner_Recorder> &rec_mng);
};