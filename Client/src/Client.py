# ============================================================
#  Project  : I-Drone
#  Filename : Client.py
#  Author   : Iván Gutiérrez
#  License  : GNU General Public License v3.0
#
#  © 2025 Iván Gutiérrez.
# ============================================================
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import socket
import struct
import threading
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'proto_bindings'))

import messages_pld_pb2
import messages_planner_pb2

class PLD_Client:
    def __init__(self, host="localhost", port=12345):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.listener_thread = None
    
    def connect(self):
        """Establishes connection to PLD server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            return False
    
    def send_config_mission(self):
        """Sends a Config_mission message to PLD"""
        config = messages_pld_pb2.Config_mission()

        config.planner_config.signal_server_config.sdf_directory = "/home/ivan/tiles"
        config.planner_config.signal_server_config.output_file = "cobertura_dbm"
        config.planner_config.signal_server_config.latitude = 40.7128
        config.planner_config.signal_server_config.longitude = -74.0060
        config.planner_config.signal_server_config.tx_height = 5.0
        config.planner_config.signal_server_config.rx_heights.extend([1.5])
        config.planner_config.signal_server_config.frequency_mhz = 900.0
        config.planner_config.signal_server_config.erp_watts = 100.0
        config.planner_config.signal_server_config.propagation_model = 1  # ITM
        config.planner_config.signal_server_config.radius = 10
        config.planner_config.signal_server_config.plot_dbm = True
        config.planner_config.signal_server_config.resolution = 0

        config.planner_config.drone_data.num_drones = 3
        targets = [
            (-74.021240, 40.727729),
            (-74.027904, 40.713574),
            (-74.005413, 40.685264),
            (-73.999582, 40.684431),
            (-73.979590, 40.700252),
            (-74.017075, 40.706080),
            (-74.040398, 40.714407),
            (-74.013743, 40.741051),
            (-74.015409, 40.740218)
        ]
        for lon, lat in targets:
            config.planner_config.drone_data.lon.append(lon)
            config.planner_config.drone_data.lat.append(lat)
        
        config.info_planner.docker_name = "Planner"
        config.info_planner.docker_file = "/home/ivan/repo/I-Drone/I-Drone/Docker/docker-compose.yml"
        config.info_planner.module_ip = "127.0.0.1"
        config.info_planner.ssh_ip = "127.0.0.1"
        config.info_planner.port = "12346"
        config.info_planner.user = "ivan"
        config.info_planner.key = "/root/.ssh/id_rsa_idrone"

        config.info_drone.docker_name = "drone_module"
        config.info_drone.docker_file = "/path/to/drone"
        config.info_drone.module_ip = ""
        config.info_drone.ssh_ip = ""
        config.info_drone.port = ""
        config.info_drone.user = ""
        config.info_drone.key = ""

        wrapper = messages_pld_pb2.WrapperFromClient()
        wrapper.config.CopyFrom(config)

        serialized = wrapper.SerializeToString()

        header = struct.pack('>I', len(serialized))
        
        try:
            self.sock.sendall(header + serialized)
            print("Config_mission sent successfully")
        except Exception as e:
            print(f"Error sending message: {e}")
    
    def listen_for_status(self):
        """Thread that listens for status messages from PLD"""
        print("Starting status listener thread...")
        while self.running:
            try:
                header_data = self._recv_exact(4)
                if not header_data:
                    break
                
                msg_length = struct.unpack('>I', header_data)[0]

                msg_data = self._recv_exact(msg_length)
                if not msg_data:
                    break

                inner_length = struct.unpack('>I', msg_data[:4])[0]
                actual_msg_data = msg_data[4:4+inner_length]

                wrapper = messages_pld_pb2.WrapperPLD()
                wrapper.ParseFromString(actual_msg_data)

                if wrapper.HasField('status'):
                    print(f"[STATUS RECEIVED] {wrapper.status.type_status}")
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Error receiving message: {e}")
                break
        
        print("Status listener thread stopped")
    
    def _recv_exact(self, num_bytes):
        """Receives exactly num_bytes from socket"""
        data = b''
        while len(data) < num_bytes:
            chunk = self.sock.recv(num_bytes - len(data))
            if not chunk:
                return None
            data += chunk
        return data
    
    def start_listening(self):
        """Starts the listening thread"""
        if self.sock:
            self.sock.settimeout(1.0)
            self.running = True
            self.listener_thread = threading.Thread(target=self.listen_for_status, daemon=True)
            self.listener_thread.start()
    
    def stop(self):
        """Stops communication and closes socket"""
        self.running = False
        if self.listener_thread:
            self.listener_thread.join(timeout=2.0)
        if self.sock:
            self.sock.close()
        print("Client stopped")

if __name__ == "__main__":
    client = PLD_Client(host="localhost", port=12345)
    
    if client.connect():
        client.start_listening()

        print("Waiting 10 seconds before sending config...")
        time.sleep(10)

        client.send_config_mission()

        try:
            print("Waiting for status messages (Ctrl+C to exit)...")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
            client.stop()
