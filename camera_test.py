import sys
import signal
from camera import *

def sig_handler(signum, frame) -> None:
    sys.exit(1)

def main():
    signal.signal(signal.SIGTERM, sig_handler)
    try:
        while True:
            get_board()
    finally:
        cleanup_camera()


if __name__ == "__main__":
    sys.exit(main())