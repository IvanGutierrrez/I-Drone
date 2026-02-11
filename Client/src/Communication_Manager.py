# ============================================================
#  Project  : I-Drone
#  Filename : Communication_Manager.py
#  Author   : Iván Gutiérrez
#  License  : GNU General Public License v3.0
#
#  © 2025 Iván Gutiérrez.
# ============================================================

import socket
import struct
import threading
import logging
import time
import json
from datetime import datetime
from Encoder_Decoder import MessageEncoderDecoder

class CommunicationManager:
    def __init__(self, host, port, status_callback=None, reconnect_interval=5.0, on_connect_callback=None, messages_received_path=None, messages_sent_path=None):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.connected = False
        self.listener_thread = None
        self.reconnect_thread = None
        self.status_callback = status_callback
        self.on_connect_callback = on_connect_callback
        self.reconnect_interval = reconnect_interval
        self.messages_received_path = messages_received_path
        self.messages_sent_path = messages_sent_path
        self.logger = logging.getLogger(__name__)
        self.pending_messages = []
        self.message_lock = threading.Lock()
        self.first_disconnect = True

    def connect(self):
        try:
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.connected = True
            self.first_disconnect = True
            self.logger.info(f"Connected to {self.host}:{self.port}")
            if self.on_connect_callback:
                self.on_connect_callback()
            return True
        except Exception:
            self.connected = False
            return False

    def _write_message_log(self, filepath, message_type, data):
        if not filepath:
            return
        try:
            timestamp = datetime.now().isoformat()
            log_entry = {
                "timestamp": timestamp,
                "type": message_type,
                "data": data
            }
            with open(filepath, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
        except Exception as e:
            self.logger.error(f"Error writing message log: {e}")

    def reconnect_loop(self):
        while self.running:
            if not self.connected:
                if self.connect():
                    self.sock.settimeout(1.0)
                    with self.message_lock:
                        for msg in self.pending_messages:
                            try:
                                self.sock.sendall(msg)
                            except:
                                pass
                        self.pending_messages.clear()
            time.sleep(self.reconnect_interval)

    def send_message(self, encoded_message):
        if not self.connected:
            with self.message_lock:
                self.pending_messages.append(encoded_message)
            return False
        try:
            self.sock.sendall(encoded_message)
            return True
        except Exception:
            was_connected = self.connected
            self.connected = False
            if was_connected and self.first_disconnect:
                self.logger.warning("Connection lost. Attempting to reconnect...")
                self.first_disconnect = False
            with self.message_lock:
                self.pending_messages.append(encoded_message)
            return False

    def send_config(self, config_mission):
        if not self.connected:
            self.logger.warning("Not connected to PLD. Try later")
        encoded = MessageEncoderDecoder.create_config_message(config_mission)
        success = self.send_message(encoded)
        if success:
            self.logger.info("Config_mission sent successfully")
            log_data = MessageEncoderDecoder.format_config_mission_for_log(config_mission)
            self._write_message_log(self.messages_sent_path, "config_mission", log_data)
        return success

    def send_command(self, command_str):
        if not self.connected:
            self.logger.warning("Not connected to PLD. Try later")
        encoded = MessageEncoderDecoder.create_command_message(command_str)
        success = self.send_message(encoded)
        if success:
            self.logger.info(f"Command '{command_str}' sent successfully")
            log_data = MessageEncoderDecoder.format_command_for_log(command_str)
            self._write_message_log(self.messages_sent_path, "command", log_data)
        return success

    def listen_for_status(self):
        self.logger.info("Starting status listener thread")
        while self.running:
            if not self.connected:
                time.sleep(1)
                continue
            try:
                header_data = self._recv_exact(4)
                if not header_data:
                    if self.connected and self.first_disconnect:
                        self.logger.warning("Connection lost. Attempting to reconnect...")
                        self.first_disconnect = False
                    self.connected = False
                    continue

                msg_length = struct.unpack('>I', header_data)[0]
                msg_data = self._recv_exact(msg_length)
                if not msg_data:
                    if self.connected and self.first_disconnect:
                        self.logger.warning("Connection lost. Attempting to reconnect...")
                        self.first_disconnect = False
                    self.connected = False
                    continue

                status = MessageEncoderDecoder.decode_status(msg_data)

                if status:
                    log_data = MessageEncoderDecoder.format_status_for_log(status)
                    self._write_message_log(self.messages_received_path, "status", log_data)
                    if self.status_callback:
                        self.status_callback(status)

            except socket.timeout:
                continue
            except Exception:
                if self.connected and self.first_disconnect:
                    self.logger.warning("Connection lost. Attempting to reconnect...")
                    self.first_disconnect = False
                self.connected = False

        self.logger.info("Status listener thread stopped")

    def _recv_exact(self, num_bytes):
        data = b''
        while len(data) < num_bytes:
            try:
                chunk = self.sock.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
            except:
                return None
        return data

    def start_listening(self, socket_timeout=1.0):
        if self.sock:
            self.sock.settimeout(socket_timeout)
        self.running = True
        self.listener_thread = threading.Thread(
            target=self.listen_for_status, 
            daemon=True
        )
        self.listener_thread.start()
        
        self.reconnect_thread = threading.Thread(
            target=self.reconnect_loop,
            daemon=True
        )
        self.reconnect_thread.start()

    def stop(self):
        self.running = False
        self.connected = False
        if self.listener_thread:
            self.listener_thread.join(timeout=2.0)
        if self.reconnect_thread:
            self.reconnect_thread.join(timeout=2.0)
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.logger.info("Communication stopped")
