# Using Custom UI 
- Make sure in `libs/config.py` the UDP_IP_TARGET matches the ip target of the Gremsy 
- In addition feel free to change RTSP_PATH to match what you set on your transport mechanism
```python
# ============================================================================
class ConnectionConfig:
    """Connection configuration settings"""
    
    # Connection methods
    CONTROL_UART = 0
    CONTROL_UDP = 1
    
    # Default connection method
    CONTROL_METHOD = CONTROL_UDP
    
    # UDP Configuration
    UDP_IP_TARGET = "192.168.1.240"      # MAKE SURE THIS IS CORRECT
    UDP_PORT_TARGET = 14566             # Do not change

    # RTSP defaults for UI streaming preview
    RTSP_PORT_TARGET = 8554
    RTSP_PATH_TARGET = "eo"
    
    # UART Configuration
    UART_PORT = "/dev/ttyUSB0"
    UART_BAUDRATE = 115200
    
    # Connection timeout
    CONNECTION_TIMEOUT = 5.0  # seconds
```