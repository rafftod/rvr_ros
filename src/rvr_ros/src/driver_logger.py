"""This script is an interface implementing logging to bypass RVR grasp on logging.

Any script that aims to log properly should refer to this class for uniformity.
"""

from datetime import datetime


class DriverLogger:
    @staticmethod
    def log(message: str) -> None:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")
