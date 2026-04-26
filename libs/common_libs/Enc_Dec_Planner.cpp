/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Planner.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Enc_Dec_Planner.h"
#include <boost/asio/detail/socket_ops.hpp>
#include "structs/Structs_Planner.h"

namespace Enc_Dec_Planner {
    std::pair<Planner, std::unique_ptr<google::protobuf::Message>> decode_to_planner(const std::string& data)
    {
        Wrapper wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {Planner::UNKNOWN, nullptr};
        }

        if (wrapper.has_planner_message()) {
            auto msg = std::make_unique<PlannerMessage>(wrapper.planner_message());
            return {Planner::ConfigMessage, std::move(msg)};
        }

        return {Planner::UNKNOWN, nullptr};
    }
        
    bool encode_config_message(const std::vector<Struct_Planner::SignalServerConfig>& signal_msgs, const Struct_Planner::DroneData& drone_msg, std::string &data) 
    {
        if (signal_msgs.empty()) {
            return false;
        }

        DroneData dron_proto;
        dron_proto.set_num_drones(drone_msg.num_drones);

        for (const auto& each : drone_msg.pos_targets) {
            dron_proto.add_lon(each.lon);
            dron_proto.add_lat(each.lat);
        }

        PlannerMessage complete_mst;
        for (const auto& signal_msg : signal_msgs) {
            auto* protoMsg = complete_mst.add_signal_server_config();

            // Required fields
            protoMsg->set_sdf_directory(signal_msg.filePaths.sdfDirectory);
            protoMsg->set_output_file(signal_msg.filePaths.outputFile);
            protoMsg->set_latitude(signal_msg.position.latitude);
            protoMsg->set_longitude(signal_msg.position.longitude);
            protoMsg->set_tx_height(signal_msg.position.txHeight);
            protoMsg->set_frequency_mhz(signal_msg.transmission.frequencyMHz);
            protoMsg->set_erp_watts(signal_msg.transmission.erpWatts);
            protoMsg->set_propagation_model(signal_msg.options.propagationModel);
            protoMsg->set_radius(signal_msg.coverage.radius);
            protoMsg->set_resolution(signal_msg.coverage.resolution);

            // Optional strings
            if (!signal_msg.filePaths.userTerrainFile.empty())
                protoMsg->set_user_terrain_file(signal_msg.filePaths.userTerrainFile);
            if (!signal_msg.filePaths.terrainBackground.empty())
                protoMsg->set_terrain_background(signal_msg.filePaths.terrainBackground);

            // Optional repeated
            for (double rxh : signal_msg.position.rxHeights)
                protoMsg->add_rx_heights(rxh);

            // Optional doubles/ints/bools
            if (signal_msg.transmission.rxThreshold != 0.0) protoMsg->set_rx_threshold(signal_msg.transmission.rxThreshold);
            if (signal_msg.transmission.horizontalPol) protoMsg->set_horizontal_pol(signal_msg.transmission.horizontalPol);
            if (signal_msg.environment.groundClutter != 0.0) protoMsg->set_ground_clutter(signal_msg.environment.groundClutter);
            if (signal_msg.environment.terrainCode != 0) protoMsg->set_terrain_code(signal_msg.environment.terrainCode);
            if (signal_msg.environment.terrainDielectric != 0.0) protoMsg->set_terrain_dielectric(signal_msg.environment.terrainDielectric);
            if (signal_msg.environment.terrainConductivity != 0.0) protoMsg->set_terrain_conductivity(signal_msg.environment.terrainConductivity);
            if (signal_msg.environment.climateCode != 0) protoMsg->set_climate_code(signal_msg.environment.climateCode);
            if (signal_msg.options.knifeEdgeDiff) protoMsg->set_knife_edge_diff(signal_msg.options.knifeEdgeDiff);
            if (signal_msg.options.win32TileNames) protoMsg->set_win32_tile_names(signal_msg.options.win32TileNames);
            if (signal_msg.options.debugMode) protoMsg->set_debug_mode(signal_msg.options.debugMode);
            if (signal_msg.options.metricUnits) protoMsg->set_metric_units(signal_msg.options.metricUnits);
            if (signal_msg.options.plotDbm) protoMsg->set_plot_dbm(signal_msg.options.plotDbm);
        }

        *(complete_mst.mutable_drone_data()) = dron_proto;

        Wrapper wrapper;
        *(wrapper.mutable_planner_message()) = complete_mst;

        std::string serialized_data;
        if (!wrapper.SerializeToString(&serialized_data))
            return false;

        uint32_t size = boost::asio::detail::socket_ops::host_to_network_long(
            static_cast<uint32_t>(serialized_data.size())
        );

        data.resize(4);
        std::memcpy(data.data(), &size, 4);
        data += serialized_data;

        return true;
    }


    bool decode_signal_server(const SignalServerConfigProto& protoMsg, Struct_Planner::SignalServerConfig &msg) 
    {
        try {
            msg.filePaths.sdfDirectory = protoMsg.sdf_directory();
            msg.filePaths.outputFile = protoMsg.output_file();
            if (protoMsg.has_user_terrain_file()) msg.filePaths.userTerrainFile = protoMsg.user_terrain_file();
            if (protoMsg.has_terrain_background()) msg.filePaths.terrainBackground = protoMsg.terrain_background();

            msg.position.latitude = protoMsg.latitude();
            msg.position.longitude = protoMsg.longitude();
            msg.position.txHeight = protoMsg.tx_height();

            msg.position.rxHeights.clear();
            for (int i = 0; i < protoMsg.rx_heights_size(); ++i)
                msg.position.rxHeights.push_back(protoMsg.rx_heights(i));
            msg.transmission.frequencyMHz = protoMsg.frequency_mhz();
            msg.transmission.erpWatts = protoMsg.erp_watts();
            if (protoMsg.has_rx_threshold()) msg.transmission.rxThreshold = protoMsg.rx_threshold();
            if (protoMsg.has_horizontal_pol()) msg.transmission.horizontalPol = protoMsg.horizontal_pol();

            if (protoMsg.has_ground_clutter()) msg.environment.groundClutter = protoMsg.ground_clutter();
            if (protoMsg.has_terrain_code()) msg.environment.terrainCode = protoMsg.terrain_code();
            if (protoMsg.has_terrain_conductivity()) msg.environment.terrainConductivity = protoMsg.terrain_conductivity();
            if (protoMsg.has_climate_code()) msg.environment.climateCode = protoMsg.climate_code();

            msg.options.propagationModel = protoMsg.propagation_model();
            if (protoMsg.has_knife_edge_diff()) msg.options.knifeEdgeDiff = protoMsg.knife_edge_diff();
            if (protoMsg.has_win32_tile_names()) msg.options.win32TileNames = protoMsg.win32_tile_names();
            if (protoMsg.has_debug_mode()) msg.options.debugMode = protoMsg.debug_mode();
            if (protoMsg.has_metric_units()) msg.options.metricUnits = protoMsg.metric_units();
            if (protoMsg.has_plot_dbm()) msg.options.plotDbm = protoMsg.plot_dbm();

            msg.coverage.radius = protoMsg.radius();
            msg.coverage.resolution = protoMsg.resolution();
        } catch (...) {
            return false;
        }

        return true;
    }

    bool decode_signal_server_list(const PlannerMessage& planner_msg, std::vector<Struct_Planner::SignalServerConfig> &msgs)
    {
        msgs.clear();
        for (int i = 0; i < planner_msg.signal_server_config_size(); ++i) {
            Struct_Planner::SignalServerConfig cfg;
            if (!decode_signal_server(planner_msg.signal_server_config(i), cfg)) {
                return false;
            }
            msgs.push_back(std::move(cfg));
        }
        return !msgs.empty();
    }

    bool decode_drone_data(const DroneData& protoMsg, Struct_Planner::DroneData &msg)
    {
        msg.num_drones = protoMsg.num_drones();
        msg.pos_targets.clear();
        for (int i = 0; i < protoMsg.lon().size(); ++i) {
            if (i >= protoMsg.lat().size())
                return false;
            Struct_Planner::Coordinate coordinate(protoMsg.lon(i),protoMsg.lat(i));
            msg.pos_targets.push_back(coordinate);
        }
        return true;
    }
};