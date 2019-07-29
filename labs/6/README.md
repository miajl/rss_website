#Lab 6: Path Planning

Our briefing slides can be found [here](https://docs.google.com/presentation/d/e/2PACX-1vSbTrn6288ExKgKn6zxwXhcQU8j9JTAl1g2xeSfVyBx5OOaQ8wHcma-ttG-lWlHgJ4d9O_niWeJlsgf/embed?start=false&loop=false&delayms=3000).

***

## **Overview and Motivations (Nada)**
For Lab 6, we had two goals: plan trajectories in an occupancy grid given a start and end pose, and program the racecar to follow a predetermined trajectory around Stata basement. Our ultimate goal was to combine these two features, such that our racecar was able to perform real-time path-planning and execution. These features were important as they are the core factors involved in autonomous driving. Without planning and control, our robot would not be able to navigate an environment efficiently and safely. With this lab, our car is finally able to reliably drive autonomously.

We tested two different approaches to path planning in this lab: search planning and sample planning. We compared these two approaches based on their efficiency and accuracy, and after considering both implementations, decided to proceed with sample planning trajectories.

Once we successfully had our racecar determining efficient paths, we implemented a pure-pursuit controller that worked with our particle filter localization from Lab 5. From here, the racecar was able to quickly, accurately, and safely follow the trajectory determined via path-planning in the real world, enabling autonomous driving.

***

## **Proposed Approach**

### *Search Planning (Nada)*
In order to implement a search-based path-planning algorithm, we decided to use A\* search on our racecar. 

Before we could do this, we needed to account for the robot's dimensions such that it would not clip any obstacles on its edges by driving too close to the walls. To avoid this situation, we dilated all obstacles on the map, such that the point representation of the robot avoiding the dilated obstacles meant that the full body of the robot would avoid the real-world obstacles successfully.

<img src="https://drive.google.com/uc?export=view&id=1_FznJNFJbGu9pMNg4KvTh-57gCy-Lcs4" alt="Path" height="462" width="583">
**Figure 1.1: Dilated Map**
Here, the map has been dilated such that all obstacles have a buffer added to them to ensure the sides of the car do not collide with or drive too close to anything.


Once we had a dilated map, we were able to implement a search algorithm. Once the robot receives clicked points in Rviz, it sets these points as its start and end points. From here, we set up a queue of neighbors to explore. This queue was ordered such that the closest neighbor to the current point was always listed first. 

At any given iteration of A\*, we looked at the node currently in the queue that had the lowest projected distance to reach the goal. We then checked if this neighbor was within a reasonable margin of the end point. If it was, we had successfully solved the problem, and could then return the trajectory, which was then published in Rviz. 

If the current node, was still too far from the goal, we expanded our search by finding all neighbors to the current node that did not interfere with an obstacle. The process for this is shown in the image below.

<img src="https://drive.google.com/uc?export=view&id=1m7anzZFBixMRwIC5-cpLYWW0RIgwRMJJ" alt="Path" height="462" width="583">
**Figure 1.2: Process of Finding Neighbor Nodes**
To find the neighbors of a node, our implementation had a set distance away from the current node that it looked for new nodes. The program then swept the car's feasible turning angles, and stepped through this angle sweep at the given radius, and set those points as the new neighbor nodes to explore.


 We then made sure these neighbors were not interfering with the locations of the obstacles, and once they had been determined to be safe, we calculated the projected distance from each of these neighbors to the goal. We then added the neighbors to the queue to be explored on a later iteration of A\*. 

During each round, we also updated the cost of each node. This cost was how much distance the path had taken to get to some node in the trajectory. This cost was updated any time a node was reached in a shorter distance than we had initially set its cost to be. In this way, we were able to ensure a shortest path would be found by this algorithm. 

<img src="https://drive.google.com/uc?export=view&id=1CecYPDg6d_jOoD1XM-EUUR1p70w17WnQ" alt="Path" height="462" width="583">
**Figure 1.3: Path Found Using A\***
Here, the trajectory that the algorithm found is seen as a blue line on the map, and the nodes that were explored to reach this trajectory are marked as green points along the way. This trajectory is represented as straight lines between the points found along the path, which ended up being very accurate since we had very small distances between any two points.


## *Sample Planning (Mia)*
### *RRT*

We chose to implement Rapidly-exploring Random Tree (RRT) for our sample based planner. RRT works by building a tree of viable paths through the state space. RRT begins each iteration by sampling a random point within the state space. To ensure that this point is a position the car could be in, it is drawn from the occupiable points on a dilated version of the map.

<img src="https://drive.google.com/uc?export=view&id=150NqzO_jKj6QjyNkfUj-I25zsY1MpCYP" alt="sample point" height="301" width="535">
**Figure 2.1: RRT Samples the State Space**

Then RRT uses a steering function to find the point the car would reach if it drove toward the sampled point from its closest graph neighbor for a specified distance.


<img src="https://drive.google.com/uc?export=view&id=1qbbPZrQpXJbRDb6iJGPIgWwh2r3pCBAf" alt="steering function" height="301" width="541">
**Figure 2.2: Steering Function Finds New Node Location**

Then if there are no obstacles from the neighbor to the new node, RRT will add the node to the graph with the neighbor as its parent.


<img src="https://drive.google.com/uc?export=view&id=1ly1Is-I8DhnAVUDqg0_4-prILTy4HOmK" alt="steering function" height="298" width="534">
**Figure 2.3: Connecting the New Node to the Graph**

After a specified number of iterations, RRT will sample in the goal region. This will make the tree reach the goal faster. Once the tree has reached the goal, the path is extracted by tracing back from the node in the goal.


<img src="https://drive.google.com/uc?export=view&id=1GkPBMY7bqKgEeRegpm6J5uJ4dpk67OkF" alt="find path" height="302" width="535">

**Figure 2.4: Path Found by RRT\***



### *RRT\* *
Because RRT does not produce optimal paths, we used the RRT\* modification, which checks for the most optimal way to connect the tree after each point is added.

RRT\* follows the same steps to find new endpoints.


<img src="https://drive.google.com/uc?export=view&id=12xej78auriIDwV_VBAyI8nGbMYSiWyOI" alt="New Node RRT*" height="268" width="435">
**Figure 2.5: New RRT\* Node**


RRT\* then looks within a search radius for the node to connect the new endpoint to such that the cost to the endpoint is minimized, checking that there are no obstacles in the way. 


<img src="https://drive.google.com/uc?export=view&id=12xej78auriIDwV_VBAyI8nGbMYSiWyOI" alt="Best neighbor RRT*" height="281" width="432">
**Figure 2.6: Connecting the New Node**


RRT\* then checks if there are any nodes within the radius that could be reached at lower cost through the new node than through its previous parent node without any obstacles in the way.


<img src="https://drive.google.com/uc?export=view&id=1d9XNlanDUGKS4Edmz7OvkJHU61jcjLtR" alt="Rewiring the Tree" height="301" width="459">
**Figure 2.7: Rewiring the Tree**


In order to avoid hitting walls we use a dilated map and use a rectangle the width of the car for obstacle detection.


##*Pure Pursuit (Eric)*
Now that we have a path from where the robot is to where we want it to be, we need to program the robot to follow that path. The approach we decided on is called pure pursuit. The robot knows its location relative to the path. The first step is to draw a circle around the car called the look-ahead circle. The radius of this circle, called the look-ahead distance, is the main factor to adjust how well the controller works. The location where the lookahead circle intersects the line is the drive point. It is possible that there is more than one point of intersection between the circle and the path. Our approach takes the point furthest along the path to be the drive point.

Once the drive point is found the car's steering angle is adjusted so that it will meet up with the drive point. As the car moves so does the lookahead circle. This means the drive point is constantly moving along the path as well. With a well tuned look-ahead distance this is a very simple and effective approach for following a path. 

To find this optimal look-ahead value we set the car's speed to 1.5 meters per second. We found this to work well with localization. Then we made a test trajectory with a sharp corner and a zig-zagged path to test to controller's ability to deal with turning. We started the are about 2 meters away from the path and repeated the experiment with five look-ahead distances: 0.5 m, 0.75 m, 1.0 m, 1.5 m, and 2.0 m. 


<img src="https://drive.google.com/uc?export=view&id=1vqgmXrXla73lJhIVAsMkdP3hYFUkyQGa" alt="Testing Path Image" height="300">
**Figure 2.8: Pure Pursuit Testing Path**

This path was designed to have an initial offset to test convergence time, a sharp turn to test response time, and a zig zag, to test noisey path performance.


<img src="https://drive.google.com/uc?export=view&id=1K864BXGs_ZxUdKpudf_kPYWV-T30jNaH" alt="Path" height="300">
**Figure 2.9: Initial Offset Error**

The convergence time was much smaller on the trial with 0.5 meters of lookahead, but it also had a larger overshoot than the 2 meter lookahead trial. 


<img src="https://drive.google.com/uc?export=view&id=1futF-wl7NFcB0QFNwpwoAf0sbJZ3IIQ2" alt="Path" height="300">
**Figure 2.10: Sharp Turn Error**


Both the 0.5 meter and 2 meter lookahead trials had large errors compared to the 1 meter trial. The large lookahead tries to smooth the curve out approximating it as a gradual curve leading to a large, long lasting error. The short lookahead reacts to the curve too late and the car cannot physically turn sharp enough to follow the desired trajectory. 1 meter of look ahead balances these two effects well. 


<img src="https://drive.google.com/uc?export=view&id=1q9vrZ4bjNagLhG_Y5gyaBsXjeFqZZPfA" alt="Path" height="300">
**Figure 2.11: Zig-Zag Error**

The zig-zag trials showed a similar result to the sharp turn trials. The long lookaheads stayed in the middle of the zig-zag and did not react to the rapidly swerving path. The short lookaheads tried to follow the path too precisely and ran into the limits of the car's cornering ability. Again 1 meter of lookahead was a good middle ground. For these reasons we decided to go with the 1 meter lookahead distance on the car.



## **Experimental Evaluation**

### *In Simulation*

### *Search Planning (Nada)*

Our search-planning approach was very good at finding shortest paths. Because our implementation explored the nodes closest to the goal first, we guaranteed that we would find the optimal path. Below, a video can be seen of our algorithm finding a path around Stata basement.  


<iframe src="https://drive.google.com/file/d/1DpyEFvg_WxxT6DW-Y7o9u-CT0ecpsScn/preview" width="640" height="480"></iframe>
**Figure 5: Search Planning in Simulation**
Here, the algorithm successfully finds an optimal path to drive around a loop in Stata basement. This was enabled using waypoints, such that the racecar would drive all the way around rather than return the shorter path that simply closed the gap between the initial and final point. This video is sped up to 2.5x -- this path took 202.8 seconds to find in real time.


However, it took a lot of time to come up with the path using this method. This implementation took about a minute of set up time in order to find all the obstacles in the map. Afterwards, it could find some paths within a few seconds, and others could take around 3 minutes to compute. 

<img src="https://drive.google.com/uc?export=view&id=1ivyzZRbTb2qM3w_XTYBvHqG1g7Egi7p0" alt="Path" height="462" width="583">
**Figure 3.1: Fast Path**
Sometimes the path was found very quickly. This path was found in 10.2 seconds. 


<img src="https://drive.google.com/uc?export=view&id=1p6uq9Lv0PSXrp2kyNfwtxET5ZNvNDF0D" alt="Path" height="462" width="583">
**Figure 3.2: Slow Path**
Other paths took a very long time to compute, such as this one. This path took 200 seconds to compute, and had to search through many nodes before it found it. This is seen in how much more dense the explored nodes marker (shown in green) appears, compared to the previous path.  


### *Sample Planning (Andrew)* 
The sample planner is able to generate acceptable paths that the car can follow successfully in significantly less time than the search planner. Though these paths are not guaranteed to be the optimal route as is the case for the search planner, the use of RRT\* for the path planning algorithm means that the generated paths are typically shorter than they would with traditional RRT. It also allows for a tradeoff between calculation time and path length, because RRT\* has parameters that can be changed to produce better paths given more time. 

To understand how different values of neighbor radius impact the computation time of RRT\* and the resulting path, tests were run on two different courses with different values and results collected. Each point represents the average result of five runs with the specified neighbor radius. 

<img src="https://drive.google.com/uc?export=view&id=1pyxsXIVd0VTfEmnFrKZ2hQlQw0caN_76" alt="RRT Neighbor radius comparison" height="366" width="597">
**Figure 4.1: Performance of different values of RRT\* neighbor radius on a long, simple path**


<img src="https://drive.google.com/uc?export=view&id=1X-ai7mpdqFqNE4vK9mv7nc72WjuCv6JG" alt="RRT Neighbor radius comparison" height="370" width="600">
**Figure 4.2: Performance of different values of RRT\* neighbor radius on a short, obstacle-filled path**


These results show that computation time and path length are strongly related through the neighbor radius. Path length quickly approaches a minimum, pseudo-optimal value while computation time increases quickly with neighbor radius. Based on these results, we picked a neighbor radius of three because it produces close to the optimal path while saving significant time when compared to higher radii. 


<img src="https://drive.google.com/uc?export=view&id=1mRNa2m4dR9J7Qdn5heqygL--PmLGwxfb" alt="RRT vs RRT\*" height="270" width="480">

**Figure 4.3: Comparison of calculation times and paths between RRT\* parameter settings**



### *On the Racecar (Andrew)* 

Our pure pursuit controller is able to follow the paths generated by our RRT\* implementation smoothly. It is able to stay on the paths without significant oscillations, and can take tight corners without losing its position.

<iframe src="https://drive.google.com/file/d/12HbXw9bdSw3BCUJkW2O0VZJYwJh0S7cK/preview" width="640" height="480"></iframe>

The video demonstrates that the car is able to follow the generated trajectory without any trouble for the majority of its length. At the end of the video the car is seen deviating from the path significantly, but this was due to a localization failure and not a problem with either the pure pursuit controller or the path itself.

## **Lessons Learned (Mia)**

The biggest thing we took away from this lab is that we need to refine the localization method. For the most part it worked well, but it loses track of the robots location if it is going fast or around turns. We also should have thought more carefully about which data structures to use earlier on because we could have had faster neighbor lookups for RRT\* if we had used an RTree rather than a more generic graph implementation.
What worked well was using a parameter for our pure pursuit controller to let it know whether it was running in the simulator or on the real car. In the past we have had issues where we would have to change multiple parameters depending on the environment instead of one simple one.
