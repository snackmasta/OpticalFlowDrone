#!/usr/bin/env python3
"""
Geofence Alarm Buzzer Listener (Wrapper for Geofence Engine)
------------------------------------------------------------
Listens for incoming UDP breach packets on port 5006 (by default).
Delegates core spatial processing, breach state tracking, and hardware PWM buzzer actuation
to `geofence_engine.py`.
"""

import sys
from geofence_engine import main

if __name__ == "__main__":
    main()
