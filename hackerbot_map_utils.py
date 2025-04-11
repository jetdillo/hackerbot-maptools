import json
import yaml
import struct
import lz4.block
import argparse
import os,sys
import serial
from PIL import Image
import cv2
import numpy as np

def list_flow_style_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)

def hex_to_bytes(hex_str: str):
    if len(hex_str) % 2 != 0:
        raise ValueError("Invalid hex string length.")

    return [int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2)]

def hex_to_grayscale(value: int,roscolor: bool) -> int:
    if value >= 0xFE:
        return 255  # White
    else:
        if roscolor:
           #0xFD is a "wall" or "obstacle" 
           if value == 0xFD:
              return 0
           else:
           #return gray for ROS compatibility
              return 128 
        elif 0x01 <= value <=0x09:
           #otherwise convert RGB values to grayscale equiv. 
            return(255 *( 0x09 - value)) // 8 
        else:   
           return 0  # Default to black for values outside range

def decompress_stream(gmstr):

    try:
       if "compressedmapdata" in gmstr:
          cmdict=json.loads(gmstr)
          cmdata=cmdict['compressedmapdata']
          process_map(cmdata)

    except json.JSONDecodeError as e:
       print(f"Error parsing GETMAP results: {e}")
       return None

def generate_ros_mapfile(mapname,res,origin,thresh_o,thresh_f):
#write a YAML file for the hackerbot map
#suitable for use with the ROS map_server
   
   mapfile_params_d = {
       "image": mapname,
       "resolution": res,
       "origin": origin, 
       "occupied_thresh": thresh_o, 
       "free_thresh": thresh_f,
       "negate": 0
   }
   
   yaml.add_representer(list, list_flow_style_representer)
   
   yaml_str = yaml.dump(mapfile_params_d, sort_keys=False)
   print(yaml_str)
   
   try:
      with open("rosmap.yaml","w") as mapfile:
         yaml.dump(mapfile_params_d,mapfile,sort_keys=False)
   except yaml.emitter.EmitterError as ye:
      print(f"Error writing mapfile {ye}")
   
#gen_mapfile("rosmap.png",0.1,[0.0,0.0,0.0],0.65,0.196)   

def decompress_mapfile(file_path):
    # Read the file content
    try:
        with open(file_path, 'r') as file:
            file_content = file.read().strip()
            #print(file_content)
    except IOError as e:
        print(f"Error reading file: {e}")
        return None
    
    # Parse the content as JSON
    try:
        data = json.loads(file_content)
    except json.JSONDecodeError as e:
        print(f"Error parsing file content as JSON: {e}")
        print(f"Assuming data is already compressed binary...")
        data=file_content
        #return None
         
    # Extract the compressedmapdata entity
    if isinstance(data, dict):
        compressed_data = data.get('compressedmapdata', '')
        process_map(compressed_data,True)
    else:
        compressed_data = data  # Assume the file contains just the hex string
        process_map(compressed_data,True)
 
    #if not compressed_data:
    #    print("No compressedmapdata found in file")
    #    return None

def apply_border(map_img):
#Apply a 1-pixel border to the decoded map
#this needs to be done with a proper numpy/OpenCV kernel but will do for a hacky POC. 

    clear=128 
    blocked=0
    unknown=255

    h,w=map_img.shape
    print(f"height={h},width={w}")

    #get all the pixels from the map that show "clear" space
    
    explored=np.argwhere(map_img == clear)
    edge_kernel=[[-1,-1],[-1,0],[1,1],[0,-1],[0,1],[1,-1],[1,0],[1,1]]
    h,w=map_img.shape

    #extract out the pixels where the map actually is
    explored=np.argwhere(map_img == 128)

    #examine N+/-1 neighbors for edges in explored vs. empty space
    #in the map to find out where to draw a boundary since the native
    #robot map assumes the edge is the boundary/wall.
    #TBD: kernel-based comparitor for image-shaped ndarrays

    for e in explored:
       cr=e[0]
       cp=e[1]
       for k in edge_kernel:
          if map_img[cr+k[0]][cp+k[1]] == 255:
             map_img[cr+k[0]][cp+k[1]] = 0

    return map_img

def process_map(lz4data,roscolor):
   
    print(len(lz4data)) 
    # Convert hex string to bytes
    try:
        byte_data = bytes.fromhex(lz4data)
    except ValueError as e:
        print(f"Error converting hex to bytes: {e}")
        return None
    
    # Parse the header (first 30 bytes)
    if len(byte_data) < 30:
        print("Data too short to contain header")
        return None
    
    # Unpack the header
    header_format = '<hiiiifff'  # int16, int, int, int, int, float, float, float
    header_size = struct.calcsize(header_format)
    
    try:
        (id_num, original_size, lz4_size, width, height, 
         resolution, origin_x, origin_y) = struct.unpack_from(header_format, byte_data)
    except struct.error as e:
        print(f"Error unpacking header: {e}")
        return None
    
    # Extract the compressed data
    compressed_cells = byte_data[header_size:header_size + lz4_size]
    
    # Decompress the data
    try:
        decompressed_cells = lz4.block.decompress(compressed_cells, uncompressed_size=original_size)
    except Exception as e:
        print(f"Error decompressing LZ4 data: {e}")
        return None
    
    # Verify decompressed size matches expected
    if len(decompressed_cells) != original_size:
        print(f"Decompressed size mismatch: expected {original_size}, got {len(decompressed_cells)}")
        return None
    
    # Create a result dictionary with all the information
    result = {
        'id': id_num,
        'original_size': original_size,
        'compressed_size': lz4_size,
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'decompressed_data': decompressed_cells,
        'grid': []
    }
   
    #Might be able to skip the PIL steps here but run with it for now.... 
    map_data = [hex_to_grayscale(cell,True) for cell in decompressed_cells]
    map_image = Image.new("L", (width,height))
    map_image.putdata(map_data)
    cv_map=np.array(map_image)
   
    bounded_map=apply_border(cv_map) 
    
    png_map = "hackerbot_map.png"
    ros_map = "hackerbot_ros_map.pgm"
    ros_map_png = "hackerbot_ros_map.png"
    map_image.save(ros_map) 
    map_image.save(png_map) 
    cv2.imwrite(ros_map,bounded_map)   
    cv2.imwrite(ros_map_png,bounded_map)   
    return result

# Example usage:
if __name__ == "__main__":

    ap = argparse.ArgumentParser(description='decode HackerBot maps for ROS ')
    ap.add_argument("-f","--file", type=str, nargs='?',
                        help="inputfile to read")
    args = ap.parse_args()

    file_path = args.file # Path to your file
    result = decompress_mapfile(file_path)
    
    if result:
        print("Successfully processed compressed map data:")
        print(f"ID: {result['id']}")
        print(f"Width: {result['width']}, Height: {result['height']}")
        print(f"Resolution: {result['resolution']} cm/pixel")
        print(f"Origin: ({result['origin_x']}, {result['origin_y']})")
        print(f"Original size: {result['original_size']} bytes")
        print(f"Compressed size: {result['compressed_size']} bytes")
        print(f"Decompressed data length: {len(result['decompressed_data'])} bytes")
