import json
import yaml
import cv2
import numpy as np
import struct
import lz4.block
import argparse
import os,sys
import serial
from PIL import Image

from hackerbot import Hackerbot

class HackerBotMapUtils:
   
   def __init__(self):
   
      self.mapdata_info={}      
      self.map_savename="hackerbot_map"
      self.map_savepath="hackerbot/maps"
      self.use_ts=False

      self.bot=Hackerbot()

   def set_mapname(self,mapname,use_timestamp=False):
       self.map_savename=mapname
       if use_timestamp:
          self.use_ts
   
   def set_outpath(self,mappath):
       self.savepath=mappath

   def _list_flow_style_representer(self,dumper, data):
       return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)

   def _hex_to_bytes(self,hex_str: str):
       if len(hex_str) % 2 != 0:
           raise ValueError("Invalid hex string length.")
   
       return [int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2)]
   
   def _hex_to_grayscale(self,value: int,roscolor: bool):
       if value >= 0xFE:
           return 255  # White
       #elif 0x01 <= value <= 0x09:
       #    return (255 * (0x09 - value)) // 8  # Scale black (09) to white (01) to grayscale
       else:
           if roscolor:
              #This flattens out areas color-coded in Tuya-generated maps
              #into a single gray space for ROS compatibility 
              return 128 
           elif 0x01 <= value <=0x09:
              #otherwise convert RGB values to grayscale equiv. 
               return(255 *( 0x09 - value)) // 8 
           else:   
              return 0  # Default to black for values outside range
   
   def decompress_stream(self,gmstr,asjson=False):
  
       if asjson: 
          try:
             if "compressedmapdata" in gmstr:
                cmdict=json.loads(gmstr)
                cmdata=cmdict['compressedmapdata']
                self.process_map(cmdata)
   
          except json.JSONDecodeError:
             #print(f"Error parsing GETMAP results: {e}")
             return None
       else:
          self.process_map(gmstr)
   
   def generate_ros_mapfile(self,mapname,res,origin,thresh_o,thresh_f):
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
       except json.JSONDecodeError:
           print(f"Error parsing file content: {e}")
           return None
       
       # Extract the compressedmapdata entity
       if isinstance(data, dict):
           compressed_data = data.get('compressedmapdata', '')
           self.process_map(compressed_data,True)
       else:
           compressed_data = data  # Assume the file contains just the hex string
           self.process_map(compressed_data,True)
    
       #if not compressed_data:
       #    print("No compressedmapdata found in file")
       #    return None
   
   def apply_border(self,map_img):
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
       print(explored) 
       #examine N+/-1 neighbors for edges in explored vs. empty space
       #in the map to find out where to draw a boundary since the native
       #robot map assumes the edge is the boundary/wall.
       #TBD: kernel-based comparitor for image-shaped ndarrays
       #Oddly, Numpy and OpenCV don't seem to have native functions that search
       #or compare, just affect
   
       for e in explored:
          cr=e[0]
          cp=e[1]
          for k in edge_kernel:
             if map_img[cr+k[0]][cp+k[1]] == 255:
                map_img[cr+k[0]][cp+k[1]] = 0
      
       return map_img
  
   
   def fetch_map(self,map_id):
   
       if self.bot.core.ping():
          self.lzmap=self.bot.base.maps.fetch(map_id)
 
   def process_map(self,roscolor,diags=False):
       print(len(self.lzmap)) 
       # Convert hex string to bytes
       try:
           byte_data = bytes.fromhex(self.lzmap)
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
       
       # pull data out of byte_data list and run through lz4 decompress
       compressed_cells = byte_data[header_size:header_size + lz4_size]
       try:
           decompressed_cells = lz4.block.decompress(compressed_cells, uncompressed_size=original_size)
       except Exception as e:
           print(f"Error decompressing LZ4 data: {e}")
           return None
       
       if len(decompressed_cells) != original_size:
           print(f"Decompressed size mismatch: expected {original_size}, got {len(decompressed_cells)}")
           return None
       
       # Diags/info JIC they asked
       if diags:
          self.mapdata_info = {
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
         
       map_data = [self._hex_to_grayscale(cell,True) for cell in decompressed_cells]
       cv_map = np.array(map_data, dtype=np.uint8).reshape((height, width))
      
       bounded_map=self.apply_border(cv_map) 
       grey_map="hackerbot_decoded_map.pgm"
       ros_map = "hackerbot_ros_map.pgm"
       ros_map_png = "hackerbot_ros_map.png"
       cv2.imwrite(grey_map,cv_map)   
       cv2.imwrite(ros_map,bounded_map)   
       cv2.imwrite(ros_map_png,bounded_map)   
	   
# Example usage:
if __name__ == "__main__":
    file_path = './map_frame.txt' # Path to your file
    hbm=HackerBotMapUtils()
    #result = mu.decompress_mapfile(file_path)
    hbm.fetch_map(1)
    hbm.process_map(roscolor=False,diags=False)
     
    if hbm.mapdata_info:
        print("Successfully processed compressed map data:")
        print(f"ID: {hbm.mapdata_info['id']}")
        print(f"Width: {hbm.mapdata_info['width']}, Height: {hbm.mapdata_info['height']}")
        print(f"Resolution: {hbm.mapdata_info['resolution']} cm/pixel")
        print(f"Origin: ({hbm.mapdata_info['origin_x']}, {hbm.mapdata_info['origin_y']})")
        print(f"Original size: {hbm.mapdata_info['original_size']} bytes")
        print(f"Compressed size: {hbm.mapdata_info['compressed_size']} bytes")
        print(f"Decompressed data length: {len(hbm.mapdata_info['decompressed_data'])} bytes")
        
        # Optionally print a small portion of the grid
        if hbm.mapdata_info['grid']:
            print("\nSample grid data (first 5x5 cells):")
            for row in hbm.mapdata_info['grid'][:5]:
                print(row[:5])
    else:
       os._exit(0)
