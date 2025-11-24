/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Algo.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Enc_Dec_Algo.h"
#include <boost/asio/detail/socket_ops.hpp>
#include "structs/Structs_Algo.h"

namespace Enc_Dec_Algo {
    std::pair<Algo, std::unique_ptr<google::protobuf::Message>> decode_to_algo(const std::string& data)
    {
        Wrapper wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {Algo::UNKNOWN, nullptr};
        }

        if (wrapper.has_algo_message()) {
            auto msg = std::make_unique<AlgorithmMessage>(wrapper.algo_message());
            return {Algo::ConfigMessage, std::move(msg)};
        }

        return {Algo::UNKNOWN, nullptr};
    }
        
    bool encode_config_message(const Struct_Algo::SignalServerConfig& signal_msg, const Struct_Algo::DroneData& drone_msg, std::string &data) 
    {
        SignalServerConfigProto protoMsg;

        // Required fields
        protoMsg.set_sdf_directory(signal_msg.sdfDirectory);
        protoMsg.set_output_file(signal_msg.outputFile);
        protoMsg.set_latitude(signal_msg.latitude);
        protoMsg.set_longitude(signal_msg.longitude);
        protoMsg.set_tx_height(signal_msg.txHeight);
        protoMsg.set_frequency_mhz(signal_msg.frequencyMHz);
        protoMsg.set_erp_watts(signal_msg.erpWatts);
        protoMsg.set_propagation_model(signal_msg.propagationModel);
        protoMsg.set_radius(signal_msg.radius);
        protoMsg.set_resolution(signal_msg.resolution);

        // Optional strings
        if (!signal_msg.userTerrainFile.empty())
            protoMsg.set_user_terrain_file(signal_msg.userTerrainFile);
        if (!signal_msg.terrainBackground.empty())
            protoMsg.set_terrain_background(signal_msg.terrainBackground);

        // Optional repeated
        for (double rxh : signal_msg.rxHeights)
            protoMsg.add_rx_heights(rxh);

        // Optional doubles/ints/bools
        if (signal_msg.rxThreshold != 0.0) protoMsg.set_rx_threshold(signal_msg.rxThreshold);
        if (signal_msg.horizontalPol) protoMsg.set_horizontal_pol(signal_msg.horizontalPol);
        if (signal_msg.groundClutter != 0.0) protoMsg.set_ground_clutter(signal_msg.groundClutter);
        if (signal_msg.terrainCode != 0) protoMsg.set_terrain_code(signal_msg.terrainCode);
        if (signal_msg.terrainDielectric != 0.0) protoMsg.set_terrain_dielectric(signal_msg.terrainDielectric);
        if (signal_msg.terrainConductivity != 0.0) protoMsg.set_terrain_conductivity(signal_msg.terrainConductivity);
        if (signal_msg.climateCode != 0) protoMsg.set_climate_code(signal_msg.climateCode);
        if (signal_msg.knifeEdgeDiff) protoMsg.set_knife_edge_diff(signal_msg.knifeEdgeDiff);
        if (signal_msg.win32TileNames) protoMsg.set_win32_tile_names(signal_msg.win32TileNames);
        if (signal_msg.debugMode) protoMsg.set_debug_mode(signal_msg.debugMode);
        if (signal_msg.metricUnits) protoMsg.set_metric_units(signal_msg.metricUnits);
        if (signal_msg.plotDbm) protoMsg.set_plot_dbm(signal_msg.plotDbm);

        DroneData dron_proto;
        dron_proto.set_num_drones(drone_msg.num_drones);

        for (const auto& each : drone_msg.pos_targets) {
            dron_proto.add_lon(each.lon);
            dron_proto.add_lat(each.lat);
        }

        AlgorithmMessage complete_mst;
        *(complete_mst.mutable_signal_server_config()) = protoMsg;
        *(complete_mst.mutable_drone_data()) = dron_proto;

        Wrapper wrapper;
        *(wrapper.mutable_algo_message()) = complete_mst;

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


    bool decode_signal_server(const SignalServerConfigProto& protoMsg, Struct_Algo::SignalServerConfig &msg) 
    {
        try {
            msg.sdfDirectory = protoMsg.sdf_directory();
            msg.outputFile = protoMsg.output_file();
            if (protoMsg.has_user_terrain_file()) msg.userTerrainFile = protoMsg.user_terrain_file();
            if (protoMsg.has_terrain_background()) msg.terrainBackground = protoMsg.terrain_background();

            msg.latitude = protoMsg.latitude();
            msg.longitude = protoMsg.longitude();
            msg.txHeight = protoMsg.tx_height();
            msg.rxHeights.clear();
            for (int i = 0; i < protoMsg.rx_heights_size(); ++i)
                msg.rxHeights.push_back(protoMsg.rx_heights(i));
            msg.frequencyMHz = protoMsg.frequency_mhz();
            msg.erpWatts = protoMsg.erp_watts();
            if (protoMsg.has_rx_threshold()) msg.rxThreshold = protoMsg.rx_threshold();
            if (protoMsg.has_horizontal_pol()) msg.horizontalPol = protoMsg.horizontal_pol();

            if (protoMsg.has_ground_clutter()) msg.groundClutter = protoMsg.ground_clutter();
            if (protoMsg.has_terrain_code()) msg.terrainCode = protoMsg.terrain_code();
            if (protoMsg.has_terrain_conductivity()) msg.terrainConductivity = protoMsg.terrain_conductivity();
            if (protoMsg.has_climate_code()) msg.climateCode = protoMsg.climate_code();

            msg.propagationModel = protoMsg.propagation_model();
            if (protoMsg.has_knife_edge_diff()) msg.knifeEdgeDiff = protoMsg.knife_edge_diff();
            if (protoMsg.has_win32_tile_names()) msg.win32TileNames = protoMsg.win32_tile_names();
            if (protoMsg.has_debug_mode()) msg.debugMode = protoMsg.debug_mode();
            if (protoMsg.has_metric_units()) msg.metricUnits = protoMsg.metric_units();
            if (protoMsg.has_plot_dbm()) msg.plotDbm = protoMsg.plot_dbm();

            msg.radius = protoMsg.radius();
            msg.resolution = protoMsg.resolution();
        } catch (...) {
            return false;
        }

        return true;
    }

    bool decode_drone_data(const DroneData& protoMsg, Struct_Algo::DroneData &msg)
    {
        msg.num_drones = protoMsg.num_drones();
        msg.pos_targets.clear();
        for (int i = 0; i < protoMsg.lon().size(); ++i) {
            if (i >= protoMsg.lat().size())
                return false;
            Struct_Algo::Coordinate coordinate(protoMsg.lon(i),protoMsg.lat(i));
            msg.pos_targets.push_back(coordinate);
        }
        return true;
    }
};