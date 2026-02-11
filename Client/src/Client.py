# ============================================================
#  Project  : I-Drone
#  Filename : Client.py
#  Author   : Iván Gutiérrez
#  License  : GNU General Public License v3.0
#
#  © 2025 Iván Gutiérrez.
# ============================================================

import logging
import os
import argparse
from Config import Config
from Communication_Manager import CommunicationManager
from Encoder_Decoder import MessageEncoderDecoder

class ClientManager:
    def __init__(self, offline_mode=False):
        self._setup_logging()
        self.logger = logging.getLogger(__name__)
        self.offline_mode = offline_mode

        self.current_status = "UNKNOWN"
        self.comm_manager = CommunicationManager(
            Config.PLD_HOST,
            Config.PLD_PORT,
            status_callback=self._on_status_received,
            reconnect_interval=Config.RECONNECT_INTERVAL,
            on_connect_callback=self._on_connected,
            messages_received_path=Config.MESSAGES_RECEIVED_PATH,
            messages_sent_path=Config.MESSAGES_SENT_PATH
        )

    def _setup_logging(self):
        log_dir = os.path.dirname(Config.LOG_PATH)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        msg_received_dir = os.path.dirname(Config.MESSAGES_RECEIVED_PATH)
        if not os.path.exists(msg_received_dir):
            os.makedirs(msg_received_dir)

        msg_sent_dir = os.path.dirname(Config.MESSAGES_SENT_PATH)
        if not os.path.exists(msg_sent_dir):
            os.makedirs(msg_sent_dir)

        logging.basicConfig(
            level=logging.INFO,
            format=Config.LOG_FORMAT,
            datefmt=Config.LOG_DATE_FORMAT,
            handlers=[
                logging.FileHandler(Config.LOG_PATH),
                logging.StreamHandler()
            ]
        )

    def _on_status_received(self, status):
        if status != self.current_status:
            self.current_status = status
            self.logger.info(f"Status changed to: {status}")

    def _on_connected(self):
        if self.offline_mode:
            self.logger.info("Offline mode: sending config automatically")
            self.send_config_from_file()

    def connect(self):
        return self.comm_manager.connect()

    def send_config_from_file(self):
        try:
            config_mission = MessageEncoderDecoder.create_config_mission_from_yaml(
                Config.CONFIG_FILE_PATH
            )
            return self.comm_manager.send_config(config_mission)
        except (FileNotFoundError, KeyError, TypeError, ValueError) as e:
            self.logger.error(f"Error loading/encoding configuration: {e}")
            self.logger.info(f"Creating example configuration file at: {Config.EXAMPLE_CONFIG_PATH}")
            MessageEncoderDecoder.create_example_config(Config.EXAMPLE_CONFIG_PATH)
            return False
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            return False

    def send_finish_command(self):
        return self.comm_manager.send_command(MessageEncoderDecoder.COMMAND_FINISH)

    def start_listening(self):
        self.comm_manager.start_listening(Config.SOCKET_TIMEOUT)

    def stop(self):
        self.comm_manager.stop()

    def run(self):
        if not self.connect():
            self.logger.warning("Failed initial connection, will retry automatically")

        self.start_listening()
        self.logger.info("Client started successfully")

        print("\n=== PLD Client Menu ===")
        print("1 - Send configuration from file")
        print("2 - Send FINISH command")
        print("q - Quit")
        print("=====================\n")

        try:
            while True:
                user_input = input().strip()
                
                if user_input == '1':
                    self.send_config_from_file()
                elif user_input == '2':
                    self.send_finish_command()
                elif user_input.lower() == 'q':
                    self.logger.info("User requested shutdown")
                    break
                else:
                    print("Invalid option. Please enter 1, 2, or q")

        except KeyboardInterrupt:
            self.logger.info("Interrupted by user (Ctrl+C)")

        finally:
            print("\nShutting down...")
            self.stop()
            self.logger.info("Client stopped")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='PLD Client')
    parser.add_argument('--offline', action='store_true', 
                        help='Offline mode: send config automatically on connect/reconnect')
    args = parser.parse_args()

    client = ClientManager(offline_mode=args.offline)
    client.run()
