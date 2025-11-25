/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Path_Cal.cpp                    
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "./Path_Cal.h"
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <sstream>
#include <algorithm>
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"
#include "common_libs/Logger.h"

using namespace operations_research;

Path_Cal::Path_Cal(const Struct_Algo::Config_struct &cnf): global_cnf_(cnf)
{
}

double Path_Cal::haversine_m(const Struct_Algo::Coordinate& a, const Struct_Algo::Coordinate& b) {
    static constexpr double R = 6371000.0;
    double dLat = (b.lat - a.lat) * M_PI / 180.0;
    double dLon = (b.lon - a.lon) * M_PI / 180.0;
    double la1 = a.lat * M_PI / 180.0;
    double la2 = b.lat * M_PI / 180.0;

    double h = sin(dLat/2)*sin(dLat/2)
             + cos(la1)*cos(la2)*sin(dLon/2)*sin(dLon/2);

    return 2 * R * asin(sqrt(h));
}

void Path_Cal::build_knn_graph(const std::vector<Struct_Algo::Coordinate>& points,
                               int k_neighbors,
                               double max_neighbor_dist_m,
                               std::vector<std::vector<std::pair<int,double>>>& adj)
{
    int n = points.size();
    adj.assign(n, {});

    for (int i = 0; i < n; i++) {

        std::vector<std::pair<double,int>> dists;
        dists.reserve(n-1);

        for (int j = 0; j < n; j++) if (i != j) {
            double d = haversine_m(points[i], points[j]);
            dists.emplace_back(d, j);
        }

        std::nth_element(
            dists.begin(),
            dists.begin() + std::min(k_neighbors, (int)dists.size()),
            dists.end()
        );

        int m = std::min(k_neighbors, (int)dists.size());

        for (int t = 0; t < m; t++) {
            auto& [dist_ij, j] = dists[t];

            if (dist_ij <= max_neighbor_dist_m)
                adj[i].emplace_back(j, dist_ij);
        }
    }

    for (int i = 0; i < n; i++) {
        for (auto& e : adj[i]) {
            int j = e.first;
            double w = e.second;

            bool found = false;
            for (auto& back : adj[j]) {
                if (back.first == i) { found = true; break; }
            }
            if (!found)
                adj[j].push_back({i, w});
        }
    }
}

std::vector<double> Path_Cal::dijkstra(int src, const std::vector<std::vector<std::pair<int,double>>>& adj)
{
    int n = adj.size();
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n, INF);

    using PQ = std::pair<double,int>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;

        for (auto& e : adj[u]) {
            int v = e.first;
            double w = e.second;

            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

std::vector<int> Path_Cal::dijkstra_path(int src, int tgt, const std::vector<std::vector<std::pair<int,double>>>& adj)
{
    int n = adj.size();
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n, INF);
    std::vector<int> prev(n, -1);

    using PQ = std::pair<double,int>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == tgt) break;

        for (auto& e : adj[u]) {
            int v = e.first;
            double w = e.second;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    std::vector<int> path;
    for (int u = tgt; u != -1; u = prev[u])
        path.push_back(u);
    std::reverse(path.begin(), path.end());
    return path;
}


void Path_Cal::compute_target_distance_matrix(const std::vector<Struct_Algo::Coordinate>& all_points,
                                              const std::vector<std::vector<std::pair<int,double>>>& adj,
                                              const std::vector<Struct_Algo::Coordinate>& pos_targets,
                                              std::vector<std::vector<int64_t>>& dist_matrix)
{
    int T = pos_targets.size();
    dist_matrix.assign(T, std::vector<int64_t>(T, 0));

    std::vector<int> closest_point(T);

    for (int t = 0; t < T; t++) {
        double best = 1e18;
        int best_i = -1;

        for (size_t i = 0; i < all_points.size(); i++) {
            double d = haversine_m(pos_targets[t], all_points[i]);
            if (d < best) {
                best = d;
                best_i = i;
            }
        }
        closest_point[t] = best_i;
    }

    for (int i = 0; i < T; i++) {
        int src_node = closest_point[i];
        auto dist = dijkstra(src_node, adj);

        for (int j = 0; j < T; j++) {
            int tgt_node = closest_point[j];
            dist_matrix[i][j] = static_cast<int64_t>(dist[tgt_node] * 1000.0);//(int64_t)dist[tgt_node];
        }
    }
}

std::vector<std::vector<Struct_Algo::Coordinate>> Path_Cal::solve_vrp(const std::vector<std::vector<int64_t>>& dist_matrix,
                                                                      const std::vector<Struct_Algo::Coordinate>& pos_targets,
                                                                      int num_drones,
                                                                      const std::vector<Struct_Algo::Coordinate> &points_cp,
                                                                      const std::vector<std::vector<std::pair<int,double>>>& adj,
                                                                      const std::shared_ptr<Algorithm_Recorder> &rec_mng)
{
    std::vector<std::vector<Struct_Algo::Coordinate>> result;

    int T = pos_targets.size();
    if (T == 0 || num_drones <= 0) return result;

    std::vector<RoutingIndexManager::NodeIndex> starts;
    std::vector<RoutingIndexManager::NodeIndex> ends;
    for (int d = 0; d < num_drones; d++) {
        int start_idx = d % T;
        starts.push_back(RoutingIndexManager::NodeIndex(start_idx));
        ends.push_back(RoutingIndexManager::NodeIndex(start_idx));
    }

    RoutingIndexManager manager(T, num_drones, starts, ends);
    RoutingModel routing(manager);

    const int transitIndex = routing.RegisterTransitCallback(
        [&](int64_t from, int64_t to) -> int64_t {
            int a = manager.IndexToNode(from).value();
            int b = manager.IndexToNode(to).value();
            return dist_matrix[a][b];
        }
    );
    routing.SetArcCostEvaluatorOfAllVehicles(transitIndex);

    int64_t max_load = static_cast<int64_t>(std::ceil(double(T) / num_drones)) + num_drones;
    routing.AddDimension(
        routing.RegisterTransitCallback([](int64_t, int64_t){ return 1; }),
        0,
        max_load,
        true,
        "load"
    );

    RoutingDimension* load_dimension = routing.GetMutableDimension("load");
    if (load_dimension) {
        load_dimension->SetGlobalSpanCostCoefficient(100);
    }

    RoutingSearchParameters params = DefaultRoutingSearchParameters();
    params.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    params.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
    params.mutable_time_limit()->set_seconds(global_cnf_.max_ortools_time);

    const Assignment* solution = routing.SolveWithParameters(params);
    if (!solution) {
        Logger::log_message(Logger::Type::ERROR, "No solution VRP found");
        return result;
    }

    Logger::log_message(Logger::Type::INFO, "Writting Or Tools result");

    std::stringstream log;
    for (int d = 0; d < num_drones; d++) {
        log << "\nDron path " << d << ":\n";
        int64_t idx = routing.Start(d);
        while (!routing.IsEnd(idx)) {
            int node = manager.IndexToNode(idx).value();
            log << "  -> (" << pos_targets[node].lat << ", " 
                      << pos_targets[node].lon << ")\n";
            idx = solution->Value(routing.NextVar(idx));
        }
        log << "  (go origin)\n";
    }
    std::string log_str = log.str();

    rec_mng->write_or_output(log_str);
    
    std::vector<int> closest_point(T);
    for (int t = 0; t < T; t++) {
        double best = 1e18;
        int best_i = -1;
        for (size_t i = 0; i < points_cp.size(); i++) {
            double d = haversine_m(pos_targets[t], points_cp[i]);
            if (d < best) { best = d; best_i = i; }
        }
        closest_point[t] = best_i;
    }

    result.resize(num_drones);

    for (int d = 0; d < num_drones; d++) {
        std::vector<int> path_target_indices;
        int64_t idx = routing.Start(d);
        while (!routing.IsEnd(idx)) {
            int node = manager.IndexToNode(idx).value();
            path_target_indices.push_back(node);
            idx = solution->Value(routing.NextVar(idx));
        }

        std::vector<Struct_Algo::Coordinate> path_full;

        for (size_t i = 0; i + 1 < path_target_indices.size(); ++i) {
            int src_target = path_target_indices[i];
            int dst_target = path_target_indices[i+1];

            int src_node = closest_point[src_target];
            int dst_node = closest_point[dst_target];

            std::vector<int> inter_nodes = dijkstra_path(src_node, dst_node, adj);
            for (int n : inter_nodes) {
                path_full.push_back(points_cp[n]);
            }
        }

        int start_node_idx = closest_point[path_target_indices.front()];
        path_full.insert(path_full.begin(), points_cp[start_node_idx]);
        path_full.push_back(points_cp[start_node_idx]);

        result[d] = std::move(path_full);
    }

    return result;
}

std::vector<size_t> Path_Cal::findNearestPoints(const std::vector<Struct_Algo::Coordinate> &points_cp, const Struct_Algo::DroneData &drone_data)
{
    std::vector<size_t> nearest_indices;  
    nearest_indices.reserve(drone_data.pos_targets.size());

    for (size_t i = drone_data.num_drones; i < drone_data.pos_targets.size(); i++) // Do not look start positions
    {
        const auto &target = drone_data.pos_targets[i];

        double bestDist = std::numeric_limits<double>::max();
        size_t bestIdx = 0;

        for (size_t j = 0; j < points_cp.size(); j++)
        {
            double d = haversine_m(target, points_cp[j]);

            if (d < bestDist) {
                bestDist = d;
                bestIdx = j;
            }
        }

        nearest_indices.push_back(bestIdx);
    }

    return nearest_indices;
}

bool Path_Cal::check_targets_signal(Struct_Algo::DroneData &drone_data, const std::vector<Struct_Algo::Coordinate> &points_cp)
{
    std::vector<size_t> nearest_points = findNearestPoints(points_cp,drone_data);
    std::vector<Struct_Algo::Coordinate> new_targets;
    for (size_t i = 0; i < drone_data.pos_targets.size(); i++) {
        if (i < static_cast<size_t>(drone_data.num_drones) || haversine_m(drone_data.pos_targets[i], points_cp[nearest_points[i - drone_data.num_drones]]) <= global_cnf_.max_distance_for_neighbor)// Do not look start positions
        {
            new_targets.push_back(drone_data.pos_targets[i]);
        } else {
            std::stringstream log;
            log << "Target (" << drone_data.pos_targets[i].lat << "," << drone_data.pos_targets[i].lon <<") deleted, below threshold";
            Logger::log_message(Logger::Type::WARNING, log.str());
        }
    }
    drone_data.pos_targets = new_targets;

    return drone_data.pos_targets.size() > static_cast<size_t>(drone_data.num_drones);
}

bool Path_Cal::calculate_path(Struct_Algo::DroneData &drone_data, std::vector<Struct_Algo::Coordinate> &points_cp, std::vector<std::vector<Struct_Algo::Coordinate>> &result, const std::shared_ptr<Algorithm_Recorder> &rec_mng)
{
    int num_drones = drone_data.num_drones;

    if (!check_targets_signal(drone_data, points_cp)) {
        Logger::log_message(Logger::Type::ERROR, "No targets with signal above threshold");
        return false;
    }

    for(auto pos : drone_data.pos_targets) {
        points_cp.push_back(pos);
    }

    std::vector<std::vector<std::pair<int,double>>> adj;
    build_knn_graph(points_cp, 
                    global_cnf_.max_neighbor,
                    global_cnf_.max_distance_for_neighbor,
                    adj);

    std::vector<std::vector<int64_t>> dist_matrix;
    compute_target_distance_matrix(points_cp, adj, drone_data.pos_targets, dist_matrix);
    
    result = solve_vrp(dist_matrix, drone_data.pos_targets, num_drones, points_cp, adj, rec_mng);

    return !result.empty();
}