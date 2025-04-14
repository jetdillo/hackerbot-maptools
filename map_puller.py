#!/bin/env python
import serial
import time
import json
from datetime import datetime
import argparse

# Helper function to send command and get response
def send_command(ser,cmd, wait_time=2):
    print(f"Sending {cmd.strip()}")
    ser.write(cmd.encode('utf-8'))
    time.sleep(wait_time)
    response = ser.read(ser.in_waiting).decode('utf-8')
    print(response)
    return response

def fetch(port,list_mode,map_id,outfile):
   #Run a "wakeup" command-sequence 
   #followed by the "GETMAP" command 
   gml = ['MACHINE,0\r\n', 'PING\r\n', 'INIT\r\n', 'MACHINE,1\r\n', 'GETML\r\n']
   ser = serial.Serial(port, baudrate=230400, timeout=1)
   map_list = []
   
   for cmd in gml:
       result = send_command(ser,cmd)
       if "OK" in result or "true" in result:
           if "map_ids" in result:
               try:
                   map_result = json.loads(result)
                   map_num = map_result['map_num']
                   map_list = map_result['map_ids']
                   print(f"Found {map_num} maps: {map_list}")
               except json.JSONDecodeError:
                   print("ERROR: Failed to parse JSON response")
               if list_mode:
                  exit(0)
       else:
           print(f"ERROR: {cmd.strip()} failed")
   
   if not map_list:
       print("ERROR: No maps available!")
       exit(1)

   if not map_id:
       print("No map specified,defaulting to first one")
       map_id=1
   
   #Let's go get us a map...
   map_id = str(map_id)
   print(f"Downloading map {map_id}...")
   
   try:
       # Send GETMAP command
       mapcmd = f"GETMAP,{map_id}\r\n"
       ser.write(mapcmd.encode('utf-8'))
       
       # Collect data with proper timeout
       mapbytes = bytearray()
       start_time = time.time()
       timeout = 10  # seconds
       
       while time.time() - start_time < timeout:
           # Read available data or wait
           chunk = ser.read(4096)  # Read up to 4KB at a time
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
       
       # Save to file with timestamp
       filename=""
       if outfile == "hbmap":
          timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
          filename = f"./hbmap_{map_id}_{timestamp}.bin"
       else:
          filename=outfile
       with open(filename, "wb") as mf:
           mf.write(mapbytes)
       return filename 
   except serial.SerialException as se:
       print(f"Serial error: {se}")
   
   #Put everything right back where we found it for the next user
   send_command("MACHINE,0\r\n")
   ser.close()

if __name__ == "__main__":

    getlist=False

    ap = argparse.ArgumentParser(description='get maps stored on a Hackerbot base')

    ap.add_argument("-o","--outfile", 
                    type=str, default="hbmap",
                    help="filename to save map as,include full path if necessary")

    ap.add_argument("-p","--port", 
                    type=str, default="/dev/ttyACM0",
                    help="serial port to use(default=/dev/ttyACM0)")

    ap.add_argument("-m","--map_id",type=int,
                    default=1,
                    help="id of map to retrieve") 

    ap.add_argument("-l","--listmaps", action='store_true',
                    help="List map ids, skip retrieval")

    args = ap.parse_args()

    result=fetch(args.port,args.listmaps,args.map_id,args.outfile)
    print(f"Map saved to {result}")

