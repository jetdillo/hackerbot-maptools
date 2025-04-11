#!/bin/env 'python'
import serial
import time
import json
gml=['MACHINE,0\r\n','PING\r\n','INIT\r\n','MACHINE,1\r\n','GETML\r\n']

ser=serial.Serial("/dev/ttyACM0",baudrate=230400,timeout=None)
ser.reset_input_buffer()

map_list = []
map_id = 0  # Default value
mapstr=''
for l in gml:
    print(f"sending {l}")
    cmdstr = l.encode('utf-8')
    ser.write(cmdstr)
    time.sleep(2)
    result = ser.read(ser.in_waiting).decode('utf-8')
    print(result)

    if "OK" in result or "true" in result:
        if "map_ids" in result:
            try:
                map_result = json.loads(result)
                print(map_result)
                map_num = map_result['map_num']
                map_list = map_result['map_ids']
                print(f"There are {map_num} maps labeled {map_list} stored on this robot")
                
                if len(map_list) > 0:
                    map_id = str(map_list[0])
                    print(f"selecting {map_id}")
                else:
                    print("Warning: No maps available in map_list!")
            except json.JSONDecodeError:
                print("ERROR: Failed to parse JSON response")
    else:
        print(f"ERROR: {l} did not process correctly")

#    ser.reset_input_buffer()
#    ser.reset_output_buffer()

# Ensure map_id is safe to use later
if not map_list:
    print("ERROR: No maps were loaded!")
else:
#Let's go get us a map!
    maplines=[]
    try:
       mapcmd="GETMAP,"+map_id+"\r\n"
       print(f"Sending {mapcmd}")
       ser.write(mapcmd.encode('utf-8'))
       time.sleep(1)
       while(ser.in_waiting>0):
          print(f"{ser.in_waiting} remaining bytes")
          mapbuf=ser.read(80)
          maplines.append(mapbuf.decode('utf-8'))
       #mapstr = ser.readline().decode('utf-8')
       mapstr = ''.join(maplines)
       print(mapstr)
       print(len(mapstr))

       with open("./active_map-"+str(map_id)+".txt","w") as mf:
          mf.write(mapstr)
    except serial.SerialException as se:
       print(f"ERROR: expected map data in response, got {se}")
#...and put everything right back where we found it...
    ser.write("MACHINE,0\r\n".encode('utf-8'))
    response=ser.read(ser.in_waiting).decode('utf-8')
    print(response) 
    ser.reset_input_buffer()
