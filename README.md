# hackerbot
code to support the Hackerbot Industries Hackerbot
hackerbot_map_utils.py  - decodes JSON-based compressed maps into ROS-compatible map files

map_puller.py - Python script to download a map off the Hackerbot to a file

maptexts - sample maps in compressed JSON format.
maps - sample map images

To run, do one of the following:

   - Use map_puller.py to download a map from your robot onto the onboard Raspi 
   - cut-and-paste the terminal output from a Python miniterm session:
     Send `MACHINE,1` followed by `GETMAP,1` (or whatever map you want) and save the whole
     JSON output from the `GETMAP,1` command to a text file. The file should have 1 continuous, very long JSON string(See the `maptexts` directory for examples)
   
   - Run `hackerbot_map_utils.py -f <textfile>`. You should see output showing the resolution of the decompressed map and number of bytes processed from the script along with a new .pgm and .png file. Those are your mapfiles. 

This is all still pretty experimental and based on a good bit of apriori assumptions, hand-decoding and general hackery. It works for me with the files included in the `maptexts` directory. It might even work for you! :) 
Please let me know about successes and failures. 
 
