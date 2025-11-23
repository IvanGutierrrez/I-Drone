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

Path_Cal::Path_Cal()
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
            dist_matrix[i][j] = (int64_t)dist[tgt_node];
        }
    }
}


void Path_Cal::solve_vrp(const std::vector<std::vector<int64_t>>& dist_matrix,
                         const std::vector<Struct_Algo::Coordinate>& pos_targets,
                         int num_drones)
{
    int T = pos_targets.size();

    std::vector<RoutingIndexManager::NodeIndex> starts(num_drones, RoutingIndexManager::NodeIndex(0));
    std::vector<RoutingIndexManager::NodeIndex> ends(num_drones, RoutingIndexManager::NodeIndex(0));

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

    RoutingSearchParameters params = DefaultRoutingSearchParameters();
    params.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    const Assignment* solution = routing.SolveWithParameters(params);
    if (!solution) {
        std::cout << "❌ No se encontró solución VRP\n";
        return;
    }

    for (int d = 0; d < num_drones; d++) {
        std::cout << "\n🚁 Ruta dron " << d << ":\n";

        int64_t idx = routing.Start(d);
        while (!routing.IsEnd(idx)) {
            int node = manager.IndexToNode(idx).value();

            std::cout << "  -> (" 
                      << pos_targets[node].lat << ", " 
                      << pos_targets[node].lon << ")\n";

            idx = solution->Value(routing.NextVar(idx));
        }
    }
}

bool Path_Cal::calculate_path(const Struct_Algo::DroneData &drone_data, const std::vector<Struct_Algo::Coordinate> &points_cp)
{
    int num_drones = drone_data.num_drones;

    std::vector<std::vector<std::pair<int,double>>> adj;
    build_knn_graph(points_cp, 
                    8, // TODO Move values to config.h
                    20.0,
                    adj);

    std::vector<std::vector<int64_t>> dist_matrix;
    compute_target_distance_matrix(points_cp, adj, drone_data.pos_targets, dist_matrix);

    solve_vrp(dist_matrix, drone_data.pos_targets, num_drones);

    return true;
}