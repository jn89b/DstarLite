# July 15th
- I'm going to need to make sure the occupancy map can adapt to the size of how I'm flying 
- Don't want to be at the edge of the map otherwise I can't really find a path outside the grid map, unless I call an expansion command 
- Want to make sure map is big enough so
- So wherever I spawn start set an offset:
  - I need to set a global map 
  - Do this by getting the min and max lat lons 
  - Based on that draw a grid based on that  
  - I have gps so I can determine my position and find my global position from the grid