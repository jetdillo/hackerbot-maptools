# hackerbot-maptools
Tool(s) to convert the onboard maps from the [Hackerbot Industries Hackerbot](https://www.hackerbot.co/) into ROS OccupancyGrid maps
<img width="1030" height="802" alt="image" src="https://github.com/user-attachments/assets/149c8e3c-915a-48a5-b001-18d268b64007" />

## What does what here:
- hackerbot_map_utils.py  - decodes JSON-based compressed maps into ROS-compatible map files
- MapPuller.py - Class for handling robot<->host I/O, map retrieval and management
- maptexts - sample maps in compressed JSON format.
- maps - sample map images


This is still kind of experimental, but has been beaten on quite a bit over the past few months since the initial release in May 2025.
There are probably more efficient/"better" ways to do some of the pixel/array torturing going on here, but I've also tried to keep the
package requirements down to just OpenCV and NumPy.


```
usage: hackerbot_map_utils.py [-h] (-f PATH | -s MAP_ID | -i | -m MAP_DUMP) [-r] [-o [OUTDIR]] [-d]

Process HackerBot obstacle maps from file or robot flash memory.

options:
  -h, --help            show this help message and exit
  -f PATH, --file PATH  use mapfile named <file> (provide full path)
  -s MAP_ID, --stored-map MAP_ID
                        use map <map_id> stored onboard robot
  -i, --inventory       List maps stored onboard robot
  -m MAP_DUMP, --map-dump MAP_DUMP
                        binary dump all stored maps
  -r, --ros-color       Use ROS OccupancyGrid color mapping (default: True)
  -o [OUTDIR], --outdir [OUTDIR]
                        output path to save to
  -d, --diags           Print diagnostic information (default: False)

```
   
   - Example:
   ```
     python hackerbot_map_utils.py -s 3 -r -d -o /home/hackerbot_user
Namespace(file=None, stored_map=3, inventory=False, map_dump=None, ros_color=True, outdir='/home/hackerbot_user', diags=True)
Found 1 maps: [3]
Getting map from robot...
Total received: 8393 bytes
original_size 39312,lz4_size 4135,width 104, height 378,resolution 0.05000000074505806,origin_x -1.4500000476837158,origin_y -8.75
Wrote /home/hackerbot_user/hackerbot_ros_map.pgm
Wrote /home/hackerbot_user/hackerbot_ros_map.png
Wrote the following rosmap.yaml:
image: rosmap.png
mode: trinary
resolution: 0.1
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

# News: February 2026
I gave a talk on this at [FOSDEM 2026](https://www.fosdem.org) :
[""Turning a cheap commercial vacuum cleaner into a useful Open Source mapping tool"](https://fosdem.org/2026/schedule/event/PDWNCJ-map-hacking-a_cheap-robot-vac-with-open-source-sw/)
