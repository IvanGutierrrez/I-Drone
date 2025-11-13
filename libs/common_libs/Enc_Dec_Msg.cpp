/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Enc_Dec_Msg.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Enc_Dec_Msg.h"
#include <boost/asio/detail/socket_ops.hpp>
#include "structs/Structs_Algo.h"

namespace Enc_Dec {
    std::pair<Algo, std::unique_ptr<google::protobuf::Message>> decode_to_algo(const std::string& data)
    {
        Wrapper wrapper;
        if (!wrapper.ParseFromString(data)) {
            return {Algo::UNKNOWN, nullptr};
        }

        if (wrapper.has_signal_server_config()) {
            auto msg = std::make_unique<SignalServerConfigProto>(wrapper.signal_server_config());
            return {Algo::SignalServerConfig, std::move(msg)};
        }

        if (wrapper.has_status()) {
            auto msg = std::make_unique<Status>(wrapper.status());
            return {Algo::Status, std::move(msg)};
        }

        return {Algo::UNKNOWN, nullptr};
    }
        
    bool encode_signal_server(const Struct_Algo::SignalServerConfig& msg, std::string &data) 
    {
        SignalServerConfigProto protoMsg;

        // Required fields
        protoMsg.set_sdf_directory(msg.sdfDirectory);
        protoMsg.set_output_file(msg.outputFile);
        protoMsg.set_latitude(msg.latitude);
        protoMsg.set_longitude(msg.longitude);
        protoMsg.set_tx_height(msg.txHeight);
        protoMsg.set_frequency_mhz(msg.frequencyMHz);
        protoMsg.set_erp_watts(msg.erpWatts);
        protoMsg.set_propagation_model(msg.propagationModel);
        protoMsg.set_radius(msg.radius);
        protoMsg.set_resolution(msg.resolution);

        // Optional strings
        if (!msg.userTerrainFile.empty())
            protoMsg.set_user_terrain_file(msg.userTerrainFile);
        if (!msg.terrainBackground.empty())
            protoMsg.set_terrain_background(msg.terrainBackground);

        // Optional repeated
        for (double rxh : msg.rxHeights)
            protoMsg.add_rx_heights(rxh);

        // Optional doubles/ints/bools
        if (msg.rxThreshold != 0.0) protoMsg.set_rx_threshold(msg.rxThreshold);
        if (msg.horizontalPol) protoMsg.set_horizontal_pol(msg.horizontalPol);
        if (msg.groundClutter != 0.0) protoMsg.set_ground_clutter(msg.groundClutter);
        if (msg.terrainCode != 0) protoMsg.set_terrain_code(msg.terrainCode);
        if (msg.terrainDielectric != 0.0) protoMsg.set_terrain_dielectric(msg.terrainDielectric);
        if (msg.terrainConductivity != 0.0) protoMsg.set_terrain_conductivity(msg.terrainConductivity);
        if (msg.climateCode != 0) protoMsg.set_climate_code(msg.climateCode);
        if (msg.knifeEdgeDiff) protoMsg.set_knife_edge_diff(msg.knifeEdgeDiff);
        if (msg.win32TileNames) protoMsg.set_win32_tile_names(msg.win32TileNames);
        if (msg.debugMode) protoMsg.set_debug_mode(msg.debugMode);
        if (msg.metricUnits) protoMsg.set_metric_units(msg.metricUnits);
        if (msg.plotDbm) protoMsg.set_plot_dbm(msg.plotDbm);

        Wrapper wrapper;
        *(wrapper.mutable_signal_server_config()) = protoMsg;

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

    bool encode_status_algo(const Struct_Algo::Status &status, std::string &message)
    {
        Wrapper wrapper;

        Status* status_msg = wrapper.mutable_status();

        std::string state_std = to_string(status);
        if (state_std.empty())
            return false;

        status_msg->set_type_status(state_std);

        std::string serialized_data;
        if (!wrapper.SerializeToString(&serialized_data))
            return false;

        uint32_t size = boost::asio::detail::socket_ops::host_to_network_long(
            static_cast<uint32_t>(serialized_data.size())
        );

        message.resize(4);
        std::memcpy(message.data(), &size, 4);
        message += serialized_data;

        return true;
    }
};