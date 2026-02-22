#!/usr/bin/env python3
from .system_main import main


# Direction map for mecanum wheels
# Order: [FL, FR, RL, RR]
DIR = {
    'Forward': (-1, -1, -1, -1),   # Forward
    'Backward': (+1, +1, +1, +1),   # Backward
    'Right': (-1, +1, -1, +1),   # Right
    'Left': (+1, -1, +1, -1),   # Left
    'RotateRight': (-1, -1, +1, +1),   # Forward Right
    'RotateLeft': ( +1, +1, -1, -1),   # Forward Left
    'Stop': (0, 0, 0, 0)    # Stop
}


if __name__ == "__main__":
    main()
