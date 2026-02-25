import json
import yaml
import struct
import lz4.block
import argparse
import serial
import os,sys
import time
import random
from datetime import datetime
from PIL import Image
import cv2
import numpy as np
import MapPuller as mp

def list_flow_style_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)

def hex_to_bytes(hex_str: str):
    if len(hex_str) % 2 != 0:
        raise ValueError("Invalid hex string length.")

    return [int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2)]

def hex_to_grayscale(value: int,roscolor: bool) -> int:
#generally quantize down color values to something useful in the context of the map being converted. 

    if value >= 0xFE:
        return 255  # White
    else:
        if roscolor:
           #0xFD is a "wall" or "obstacle" 
           if value == 0xFD:
              return 0
           else:
           #flush local coloring
           #ROS technically considers white to be "unoccupied" but since the native map 
           #has no concept of a border for the map, we have to use "unexplored" grey first
           #we'll come back this once we have actual borders established
              return 128 
        elif 0x01 <= value <=0x09:
           #otherwise convert RGB values to grayscale equiv. 
            return(255 *( 0x09 - value)) // 8 
        else:   
           return 0  # Default to black for everything else.
                     # Not likely to represent an obstacle and therefore not useful in a ROS context

def generate_ros_yaml(mapname,res,origin,thresh_o,thresh_f):
#write a YAML file for the hackerbot map
#suitable for use with the ROS map_server
   
   mapfile_params_d = {
       "image": mapname,
       "mode": "trinary",
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


def get_map_from_robot(map_id):
    map_json=''
    cmdata=''
    mh = mp.MapPuller()
    mh.connect()
    _,map_l= mh.get_map_list() 
    
    if map_id in map_l: 
       try:
          map_json=mh.get_map_data(map_id).decode('utf-8').strip() 
       except Exception as e:
          raise RuntimeError(f"Error getting map data: {e}")
    try:
       if "compressedmapdata" in map_json:
          cmdict=json.loads(map_json)
          cmdata=cmdict['compressedmapdata']
    
    except json.JSONDecodeError as e:
       print(f"Error parsing GETMAP results: {e}")
       return None

    return cmdata

def dump_robot_maps():
    mh = mp.MapPuller()
    mh.connect()
    _,map_l = mh.get_map_list()
    
    for m in map_l:
       try:
          map_bytes=get_map_data(m)
          with open("./active_map-"+str(map_id)+".txt","wb") as mf:
             mf.write(map_bytes)
      
       except Exception as e:
          raise RuntimeError(f"Error getting map data for map {m}")
    
def get_map_from_file(file_path):
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
         
    # Extract the compressedmapdata entity
    if isinstance(data, dict):
        compressed_data = data.get('compressedmapdata', '')
    else:
        compressed_data = data  # Assume the file contains just the hex string
      #  process_map(compressed_data,True)
 
    #if not compressed_data:
    #    print("No compressedmapdata found in file")
    #    return None
    return compressed_data

def apply_border(map_img):
#Apply a 1-pixel border to the decoded map

    clear=128 
    blocked=0
    unknown=255

    h,w=map_img.shape
    print(f"height={h},width={w}")

    #get all the pixels from the map that show "clear" space
   
    mask = (map_img == clear)
    #pad around the mask
    border_pad = np.pad(mask, 1, constant_values=False)
    #consider neighboring pixels on surrounding 6 sides
    neighbors = (border_pad[:-2,1:-1] | border_pad[2:,1:-1] | border_pad[1:-1,:-2] | 
             border_pad[1:-1,2:] | border_pad[:-2,:-2] | border_pad[2:,2:] | 
             border_pad[:-2,2:] | border_pad[2:,:-2])
    border = neighbors & (map_img == 255)

    #one-shot border around the selected pixels
    map_img[border] = blocked
    
    return map_img
    
def map_to_occupancy(map_img,unexp=128,clear=255,sub_size=10):
#This flood-fills the map, preserving borders but erasing low-confidence obstacles
#unexp - color value for unexplored regions
#clear - color value for explored open space
#sub_size - size of subregion to examine for coincidence of border,inside and outside areas

    h,w = map_img.shape[:2]
    
    #internal_color = (255,255,255) 
    #external_color = (128,128,128) 
    internal_color = clear
    external_color = unexp 
    
    #Get all the border pixels
    #Assume tiny maps and tiny robots just aren't a thing 
    border_vec=np.argwhere(map_img==0)
    if len(border_vec) < 100:
       print("Occupancy Grid Failed: Map too small!")
       sys.exit()
    else:
       bv_r=0
       bv_c=0 
       bp=0 
       inseed=[]
       outseed=[]
       found_border = False 
       submap=np.zeros((sub_size,sub_size,3))
       tries=0
       while not found_border:
          bp = np.random.randint(len(border_vec))
          bv_r , bv_c = border_vec[bp][:2]
          print(f"trying {bv_r-sub_size}:{bv_c-sub_size},{bv_r+sub_size}:{bv_c+sub_size}")
          submap=map_img[bv_r-sub_size:bv_c-sub_size,bv_r+sub_size:bv_c+sub_size]

          #Match exactly grey/white
          #then scrabble around in the guts of the map_img
          if np.any(submap==128) and np.any(submap==255):
             found_border=True 

             #ugly and slow but the submap is tiny 

             for y in range(bv_r-10,bv_r+10):
                for x in range(bv_c-10,bv_c+10):
                   print(map_img.shape)
                   if map_img[y,x] == 128:
                      inseed=[y,x]
                   if map_img[y,x] == 255:
                      outseed=[y,x]
    
                   if len(inseed) >0 and len(outseed) >0: 
                      break
             print(f"inseed = {inseed} outseed = {outseed}")
    
          else:
             tries+=1
       if len(inseed) > 0 and len(outseed) >0: 
          h, w = map_img.shape[:2]
          mask = np.zeros((h + 2, w + 2), np.uint8)
    
          # Flood fill with flags
          flags = 4  # 4 or 8 connectivity
          flags |= cv2.FLOODFILL_FIXED_RANGE  # Use fixed range
          flags |= (255 << 8)  # Fill mask with 255
          
          lo_diff = (5, 5, 5)  # Lower difference
          up_diff = (5, 5, 5)  # Upper difference
    
    # Perform flood fill
          
          cv2.floodFill(map_img, mask, inseed, internal_color, lo_diff, up_diff, flags)
          cv2.floodFill(map_img, mask, outseed, external_color, lo_diff,up_diff, flags)
          cv2.imwrite('flood_filled_result.jpg', map_img)  
          return map_img
    
def process_map(lz4data,roscolor):
    byte_data=bytearray(b'')
    print(type(lz4data))
    if type(lz4data) is str:
       try:
           byte_data = bytes.fromhex(lz4data)
       except ValueError as e:
           print(f"Error converting hex to bytes: {e}")
           return None
    else:
       byte_data = lz4data[:]
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

    print(f"original_size {original_size},lz4_size {lz4_size},width {width}, height {height},resolution {resolution},origin_x {origin_x},origin_y {origin_y}") 
    # Extract the compressed data
    compressed_cells = byte_data[header_size:header_size + lz4_size]
    
    # Decompress the data
    try:
        decompressed_cells = lz4.block.decompress(compressed_cells, uncompressed_size=original_size)
    except Exception as e:
        print(len(compressed_cells))
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
    if roscolor:
       return map_to_occupancy(bounded_map)
    else:
       return bounded_map

def write_map(map_image,outpath):
    ros_map = outpath+"/"+"hackerbot_ros_map.pgm"
    ros_map_png = outpath+"/"+"hackerbot_ros_map.png"
    cv2.imwrite(ros_map,map_image)   
    cv2.imwrite(ros_map_png,map_image)   
    generate_ros_yaml("rosmap.png",0.1,[0.0,0.0,0.0],0.65,0.196)   

def main():  
    ap = argparse.ArgumentParser(
        description='Process HackerBot obstacle maps from file or robot flash memory.'
    )

    #choose and perish: Either get the map data from a locally-saved file 
    #or retrieve it from the robot serial port
    ap_group = ap.add_mutually_exclusive_group(required=True)

    ap_group.add_argument('-f','--file',
        type=str,
        metavar='PATH',
        help='use mapfile named <file> (provide full path)'
    )

    ap_group.add_argument('-s','--stored-map',
        type=int,
        metavar='MAP_ID',
        help='use map <map_id> stored onboard robot'
    )
    ap_group.add_argument('-i','--inventory',
        action='store_true',
        help='List maps stored onboard robot'
    )

    ap_group.add_argument('-m','--map-dump',
        help='binary dump all stored maps' 
    )

    # Optional arguments
    ap.add_argument('-r', '--ros-color',
        action='store_true',
        default=True,
        help='Use ROS OccupancyGrid color mapping (default: True)'
    )
    
    ap.add_argument("-o","--outdir",
        type=str,
        nargs='?',
        default=os.environ.get('HOME'),    
        help="output path to save to"
    ) 

    ap.add_argument('-d','--diags',
        action='store_true',
        default=False,
        help='Print diagnostic information (default: False)'
    )

    args = ap.parse_args()
    print(args)
    if args.inventory:
       print("Getting map inventory, please wait...")
       mh = mp.MapPuller()
       mh.connect()
       mapcount,map_l= mh.get_map_list() 
       print(f"robot currently has {mapcount} maps stored:\n{map_l}")
       sys.exit()       
    
    if args.map_dump:
       mh = mp.MapPuller()
       mh.connect()
       

    if args.stored_map:
       map_id = args.stored_map
       mapdata = get_map_from_robot(map_id)
       
    elif args.file:
       map_path = args.file
       mapdata = get_map_from_file(args.file) 

    map_image=process_map(mapdata,args.ros_color)
    write_map(map_image,args.outdir)
   

if __name__ == "__main__":
   main()
