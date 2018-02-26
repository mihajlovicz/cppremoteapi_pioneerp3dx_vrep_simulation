

Cppremoteapi for pioneer p3dx robot simulation

Model of the robot uses point cloud sensor to detect obstacles. Data is analyzed with several PCL library (Point Cloud Library) functions 
in order to calculate obstacle positions. This is then used with RVO2 library (Reciprocal Collision Avoidance) to calculate speed
vectors necessary to avoid obstacles.
