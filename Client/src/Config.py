# ============================================================
#  Project  : I-Drone
#  Filename : Config.py
#  Author   : Iván Gutiérrez
#  License  : GNU General Public License v3.0
#
#  © 2025 Iván Gutiérrez.
# ============================================================

import os

class Config:
    PLD_HOST = "localhost"
    PLD_PORT = 12345

    CONFIG_FILE_PATH = os.path.join(
        os.path.dirname(__file__), 
        "..", 
        "config", 
        "mission_config.yml"
    )

    EXAMPLE_CONFIG_PATH = os.path.splitext(CONFIG_FILE_PATH)[0] + "_example" + os.path.splitext(CONFIG_FILE_PATH)[1]

    LOG_PATH = os.path.join(
        os.path.dirname(__file__), 
        "..", 
        "logs", 
        "client.log"
    )

    MESSAGES_RECEIVED_PATH = os.path.join(
        os.path.dirname(__file__), 
        "..", 
        "logs", 
        "messages_received.json"
    )

    MESSAGES_SENT_PATH = os.path.join(
        os.path.dirname(__file__), 
        "..", 
        "logs", 
        "messages_sent.json"
    )

    LOG_FORMAT = "%(asctime)s  [%(levelname)-8s] %(message)s"
    LOG_DATE_FORMAT = "%d/%m/%YT%H:%M:%S"
    SOCKET_TIMEOUT = 1.0
    RECONNECT_INTERVAL = 5.0
