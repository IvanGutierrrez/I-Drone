# ============================================================
#  Project  : I-Drone
#  Filename : Encoder_Decoder.py
#  Author   : Iván Gutiérrez
#  License  : GNU General Public License v3.0
#
#  © 2025 Iván Gutiérrez.
# ============================================================

import struct
import sys
import os
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'proto_bindings'))

import messages_pld_pb2
import messages_planner_pb2

class MessageEncoderDecoder:
    COMMAND_FINISH = "FINISH"

    @staticmethod
    def encode_message(message):
        serialized = message.SerializeToString()
        header = struct.pack('>I', len(serialized))
        return header + serialized

    @staticmethod
    def decode_status(data):
        try:
            if len(data) > 4:
                inner_length = struct.unpack('>I', data[:4])[0]
                actual_msg_data = data[4:4+inner_length]
            else:
                actual_msg_data = data

            wrapper = messages_pld_pb2.WrapperPLD()
            wrapper.ParseFromString(actual_msg_data)

            if wrapper.HasField('status'):
                return wrapper.status.type_status
            return None
        except Exception as e:
            raise Exception(f"Error decoding status: {e}")

    @staticmethod
    def create_config_mission_from_yaml(yaml_path):
        with open(yaml_path, 'r') as f:
            config_data = yaml.safe_load(f)

        config = messages_pld_pb2.Config_mission()

        ss_config = config_data['planner_config']['signal_server_config']
        config.planner_config.signal_server_config.sdf_directory = ss_config['sdf_directory']
        config.planner_config.signal_server_config.output_file = ss_config['output_file']
        config.planner_config.signal_server_config.latitude = ss_config['latitude']
        config.planner_config.signal_server_config.longitude = ss_config['longitude']
        config.planner_config.signal_server_config.tx_height = ss_config['tx_height']
        config.planner_config.signal_server_config.rx_heights.extend(ss_config['rx_heights'])
        config.planner_config.signal_server_config.frequency_mhz = ss_config['frequency_mhz']
        config.planner_config.signal_server_config.erp_watts = ss_config['erp_watts']
        config.planner_config.signal_server_config.propagation_model = ss_config['propagation_model']
        config.planner_config.signal_server_config.radius = ss_config['radius']
        config.planner_config.signal_server_config.resolution = ss_config['resolution']

        if 'user_terrain_file' in ss_config and ss_config['user_terrain_file']:
            config.planner_config.signal_server_config.user_terrain_file = ss_config['user_terrain_file']
        if 'terrain_background' in ss_config and ss_config['terrain_background']:
            config.planner_config.signal_server_config.terrain_background = ss_config['terrain_background']
        if 'rx_threshold' in ss_config and ss_config['rx_threshold'] is not None:
            config.planner_config.signal_server_config.rx_threshold = ss_config['rx_threshold']
        if 'horizontal_pol' in ss_config and ss_config['horizontal_pol'] is not None:
            config.planner_config.signal_server_config.horizontal_pol = ss_config['horizontal_pol']
        if 'ground_clutter' in ss_config and ss_config['ground_clutter'] is not None:
            config.planner_config.signal_server_config.ground_clutter = ss_config['ground_clutter']
        if 'terrain_code' in ss_config and ss_config['terrain_code'] is not None:
            config.planner_config.signal_server_config.terrain_code = ss_config['terrain_code']
        if 'terrain_dielectric' in ss_config and ss_config['terrain_dielectric'] is not None:
            config.planner_config.signal_server_config.terrain_dielectric = ss_config['terrain_dielectric']
        if 'terrain_conductivity' in ss_config and ss_config['terrain_conductivity'] is not None:
            config.planner_config.signal_server_config.terrain_conductivity = ss_config['terrain_conductivity']
        if 'climate_code' in ss_config and ss_config['climate_code'] is not None:
            config.planner_config.signal_server_config.climate_code = ss_config['climate_code']
        if 'knife_edge_diff' in ss_config and ss_config['knife_edge_diff'] is not None:
            config.planner_config.signal_server_config.knife_edge_diff = ss_config['knife_edge_diff']
        if 'win32_tile_names' in ss_config and ss_config['win32_tile_names'] is not None:
            config.planner_config.signal_server_config.win32_tile_names = ss_config['win32_tile_names']
        if 'debug_mode' in ss_config and ss_config['debug_mode'] is not None:
            config.planner_config.signal_server_config.debug_mode = ss_config['debug_mode']
        if 'metric_units' in ss_config and ss_config['metric_units'] is not None:
            config.planner_config.signal_server_config.metric_units = ss_config['metric_units']
        if 'plot_dbm' in ss_config and ss_config['plot_dbm'] is not None:
            config.planner_config.signal_server_config.plot_dbm = ss_config['plot_dbm']

        drone_data = config_data['planner_config']['drone_data']
        config.planner_config.drone_data.num_drones = drone_data['num_drones']
        for target in drone_data['targets']:
            config.planner_config.drone_data.lon.append(target['lon'])
            config.planner_config.drone_data.lat.append(target['lat'])

        info_planner = config_data['info_planner']
        config.info_planner.docker_name = info_planner['docker_name']
        config.info_planner.docker_file = info_planner['docker_file']
        config.info_planner.module_ip = info_planner['module_ip']
        config.info_planner.ssh_ip = info_planner['ssh_ip']
        config.info_planner.port = info_planner['port']
        config.info_planner.user = info_planner['user']
        config.info_planner.key = info_planner['key']

        info_drone = config_data['info_drone']
        config.info_drone.docker_name = info_drone['docker_name']
        config.info_drone.docker_file = info_drone['docker_file']
        config.info_drone.module_ip = info_drone['module_ip']
        config.info_drone.ssh_ip = info_drone['ssh_ip']
        config.info_drone.port = info_drone['port']
        config.info_drone.user = info_drone['user']
        config.info_drone.key = info_drone['key']

        config.drone_sim = config_data['drone_sim']

        return config

    @staticmethod
    def create_example_config(example_path):
        example_content = """# ============================================================
#  Project  : I-Drone
#  Example Configuration File
#  Required fields are marked as such
#  Optional fields are marked with # OPTIONAL
# ============================================================

planner_config:
  signal_server_config:
    sdf_directory: ""  # REQUIRED
    output_file: ""  # REQUIRED
    user_terrain_file: ""  # OPTIONAL
    terrain_background: ""  # OPTIONAL
    latitude: 0.0  # REQUIRED
    longitude: 0.0  # REQUIRED
    tx_height: 0.0  # REQUIRED
    rx_heights: []  # REQUIRED (list of doubles)
    frequency_mhz: 0.0  # REQUIRED
    erp_watts: 0.0  # REQUIRED
    rx_threshold: 0.0  # OPTIONAL
    horizontal_pol: false  # OPTIONAL
    ground_clutter: 0.0  # OPTIONAL
    terrain_code: 0  # OPTIONAL
    terrain_dielectric: 0.0  # OPTIONAL
    terrain_conductivity: 0.0  # OPTIONAL
    climate_code: 0  # OPTIONAL
    propagation_model: 0  # REQUIRED
    knife_edge_diff: false  # OPTIONAL
    win32_tile_names: false  # OPTIONAL
    debug_mode: false  # OPTIONAL
    metric_units: false  # OPTIONAL
    plot_dbm: false  # OPTIONAL
    radius: 0.0  # REQUIRED
    resolution: 0  # REQUIRED

  drone_data:
    num_drones: 0  # REQUIRED
    targets:  # REQUIRED
      - lon: 0.0
        lat: 0.0

info_planner:  # REQUIRED
  docker_name: ""
  docker_file: ""
  module_ip: ""
  ssh_ip: ""
  port: ""
  user: ""
  key: ""

info_drone:  # REQUIRED
  docker_name: ""
  docker_file: ""
  module_ip: ""
  ssh_ip: ""
  port: ""
  user: ""
  key: ""

drone_sim: ""  # REQUIRED
"""
        os.makedirs(os.path.dirname(example_path), exist_ok=True)
        with open(example_path, 'w') as f:
            f.write(example_content)

    @staticmethod
    def create_config_message(config_mission):
        wrapper = messages_pld_pb2.WrapperFromClient()
        wrapper.config.CopyFrom(config_mission)
        return MessageEncoderDecoder.encode_message(wrapper)

    @staticmethod
    def create_command_message(command_str):
        wrapper = messages_pld_pb2.WrapperFromClient()
        wrapper.message.command = command_str
        return MessageEncoderDecoder.encode_message(wrapper)

    @staticmethod
    def format_config_mission_for_log(config_mission):
        ss_config = config_mission.planner_config.signal_server_config
        drone_data = config_mission.planner_config.drone_data

        targets = []
        for i in range(len(drone_data.lon)):
            targets.append({
                "lon": drone_data.lon[i],
                "lat": drone_data.lat[i]
            })

        signal_config = {
            "sdf_directory": ss_config.sdf_directory,
            "output_file": ss_config.output_file,
            "latitude": ss_config.latitude,
            "longitude": ss_config.longitude,
            "tx_height": ss_config.tx_height,
            "rx_heights": list(ss_config.rx_heights),
            "frequency_mhz": ss_config.frequency_mhz,
            "erp_watts": ss_config.erp_watts,
            "propagation_model": ss_config.propagation_model,
            "radius": ss_config.radius,
            "resolution": ss_config.resolution
        }

        if ss_config.HasField('user_terrain_file'):
            signal_config['user_terrain_file'] = ss_config.user_terrain_file
        if ss_config.HasField('terrain_background'):
            signal_config['terrain_background'] = ss_config.terrain_background
        if ss_config.HasField('rx_threshold'):
            signal_config['rx_threshold'] = ss_config.rx_threshold
        if ss_config.HasField('horizontal_pol'):
            signal_config['horizontal_pol'] = ss_config.horizontal_pol
        if ss_config.HasField('ground_clutter'):
            signal_config['ground_clutter'] = ss_config.ground_clutter
        if ss_config.HasField('terrain_code'):
            signal_config['terrain_code'] = ss_config.terrain_code
        if ss_config.HasField('terrain_dielectric'):
            signal_config['terrain_dielectric'] = ss_config.terrain_dielectric
        if ss_config.HasField('terrain_conductivity'):
            signal_config['terrain_conductivity'] = ss_config.terrain_conductivity
        if ss_config.HasField('climate_code'):
            signal_config['climate_code'] = ss_config.climate_code
        if ss_config.HasField('knife_edge_diff'):
            signal_config['knife_edge_diff'] = ss_config.knife_edge_diff
        if ss_config.HasField('win32_tile_names'):
            signal_config['win32_tile_names'] = ss_config.win32_tile_names
        if ss_config.HasField('debug_mode'):
            signal_config['debug_mode'] = ss_config.debug_mode
        if ss_config.HasField('metric_units'):
            signal_config['metric_units'] = ss_config.metric_units
        if ss_config.HasField('plot_dbm'):
            signal_config['plot_dbm'] = ss_config.plot_dbm

        return {
            "planner_config": {
                "signal_server_config": signal_config,
                "drone_data": {
                    "num_drones": drone_data.num_drones,
                    "targets": targets
                }
            },
            "info_planner": {
                "docker_name": config_mission.info_planner.docker_name,
                "docker_file": config_mission.info_planner.docker_file,
                "module_ip": config_mission.info_planner.module_ip,
                "ssh_ip": config_mission.info_planner.ssh_ip,
                "port": config_mission.info_planner.port,
                "user": config_mission.info_planner.user,
                "key": config_mission.info_planner.key
            },
            "info_drone": {
                "docker_name": config_mission.info_drone.docker_name,
                "docker_file": config_mission.info_drone.docker_file,
                "module_ip": config_mission.info_drone.module_ip,
                "ssh_ip": config_mission.info_drone.ssh_ip,
                "port": config_mission.info_drone.port,
                "user": config_mission.info_drone.user,
                "key": config_mission.info_drone.key
            },
            "drone_sim": config_mission.drone_sim
        }

    @staticmethod
    def format_command_for_log(command_str):
        return {"command": command_str}

    @staticmethod
    def format_status_for_log(status_str):
        return {"status": status_str}
