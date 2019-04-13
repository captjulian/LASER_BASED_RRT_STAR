This is a readme file for LASER_BASED_REALTIME_RRTSTAR algorithm.

Before using this package you should install AEROSTACK and put this package in the fold of stack_devel.

the way of installing AEROSTACK can be seen from https://github.com/Vision4UAV/Aerostack/wiki/Install-Aerostack.

1.launch file : laser_based_rrt_star.launch

before run the algorithm, you should choose 4 points for the area of navigation and 1 point for the position of drone 

2.start point and end point

start point will always be the positin of drone
end point will be a random point generate by the code

3.rrt_star_perform is the main function of rrtstar algorithm.

4.using signed distance map to detect the obstacles and the size of the signed distace map is 4mX4m(the resolution is 0.02m).

A description of the algorithm can be seen.

![Image text](https://github.com/captjulian/LASER_BASED_RRT_STAR/blob/master/LASER_BASED_RRT_STAR/process.png)
