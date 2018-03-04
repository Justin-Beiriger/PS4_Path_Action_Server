# my_path_action_server

This package contains the implementation of an action server - action client communication framework in which the client sends a path request to the server and the server executes the paths. If the lidar alarm goes off, the client cancels the path, performs a rotation to avoid a collision, and resends a path that begins where the server left off (using the feedback from the server to know which poses along the path were completed). 

## Example usage

The nodes were executed and functionality was demonstrated on the STDR simulator. 

## Running tests/demos

The video file showing the demo can be found at the following link:

