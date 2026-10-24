# DIY NMEA 2000 Digital Switching Node (ESP32-S3)

Custom helm digital switching and monitoring node built for small boats using the Waveshare ESP32-S3 Industrial 8-Channel Relay Board.

## Features
- **NMEA 2000 Integration:** Uses PGN 127501 (Binary Status) and PGN 127502 (Binary Control).
- **Dual-Bank Instance Architecture:** 
  - Bank 0: Relay Outputs (Cabin Light, Anchor Lights, Nav Lights, Dash Lights, Wipers, Spares)
  - Bank 1: Sensor & Alarm Inputs (High-Water Alarm, Bilge Running, Port Drive Tilt, Starboard Drive Tilt)
- **Persistent State Storage:** Uses ESP32 Flash (`Preferences.h`) to remember relay states across power cycles.
- **Safety Guards:** Blocks MFD control commands from modifying read-only sensor channels.
