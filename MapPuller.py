#!/bin/env python
import serial
import time
import json


class MapPuller:
    #Class for retrieving map data from a Hackerbot base via serial connection.
    
    def __init__(self, port="/dev/ttyACM0", baudrate=230400, timeout=1):
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
    
    #serial command interface to robot
    def _send_command(self, cmd, wait_time=2,debug=False):
 
        self.ser.write(cmd.encode('utf-8'))
        time.sleep(wait_time)
        response = self.ser.read(self.ser.in_waiting).decode('utf-8')
        if debug: 
           print(f"Sent {cmd.strip()}")
           print(response)
        return response
    
    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout)
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self._send_command("MACHINE,0\r\n")
            self.ser.close()
    
    def get_map_list(self):
        # sending the whole command sequence JIC
        commands = ['MACHINE,0\r\n', 'PING\r\n', 'INIT\r\n', 'MACHINE,1\r\n', 'GETML\r\n']
        
        for cmd in commands:
            result = self._send_command(cmd)
            if "OK" in result or "true" in result:
                if "map_ids" in result:
                    try:
                        map_result = json.loads(result)
                        map_num = map_result['map_num']
                        map_ids = map_result['map_ids']
                        print(f"Found {map_num} maps: {map_ids}")
                        return map_num, map_ids
                    except json.JSONDecodeError:
                        raise RuntimeError("Failed to parse JSON response")
            else:
                print(f"ERROR: {cmd.strip()} failed")
        
        raise RuntimeError("No maps available!")
    
    # Download a single map from the robot
    def get_map_data(self, map_id=None, download_timeout=10):
        # Get the first map if map_id not specified
        if map_id is None:
            _, map_list = self.get_map_list()
            if map_list:
                map_id = map_list[0]
                print(f"No map specified, defaulting to first one: {map_id}")
            else:
                raise RuntimeError("No maps available!")
        
        # Download the map
        map_id = str(map_id)
        try:
            # Send GETMAP command
            mapcmd = f"GETMAP,{map_id}\r\n"
            self.ser.write(mapcmd.encode('utf-8'))
            
            # Collect data with proper timeout
            mapbytes = bytearray()
            start_time = time.time()
            
            while time.time() - start_time < download_timeout:
                # Read available data
                chunk = self.ser.read(4096)  # Read up to 4KB at a time
                if chunk:
                    mapbytes.extend(chunk)
                    print(f"Received {len(mapbytes)} bytes", end='\r')
                    
                    # Check for JSON end if format is known
                    if mapbytes.endswith(b'}'):
                        break
                else:
                    # No data received, check if we should continue waiting
                    if len(mapbytes) > 0:  # Already got some data
                        break
            
            print(f"\nTotal received: {len(mapbytes)} bytes")
            return mapbytes
            
        except serial.SerialException as se:
            raise RuntimeError(f"Serial error: {se}")
    
    def __enter__(self):
        # Context manager entry.
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        # Context manager exit.
        self.disconnect()
