# Multi-Objective Path Optimization System (A* and ACO Algorithm)

## Overview

This project is a MATLAB-based system for multi-objective path optimization that integrates both the A* search algorithm and Ant Colony Optimization (ACO) algorithm. The system is designed to find the optimal path in a grid environment with obstacles while considering multiple objectives such as path length, obstacle avoidance, and dynamic target adjustments.

## Features

- **A* Search Algorithm**:
  - A* algorithm computes the shortest path between a start point and a target point by expanding nodes based on the cost function (distance from start and heuristic to goal).
  
- **Ant Colony Optimization (ACO)**:
  - ACO is used to optimize the sequence of visiting multiple target points.
  
- **Dynamic Obstacle Handling**:
  - The system dynamically adjusts the path to avoid obstacles in real time, considering both static and dynamic obstacles.

- **Multi-Objective**:
  - The system considers multiple criteria for path optimization, including path length, smoothness, and avoidance of both static and dynamic obstacles.

## File Descriptions

### Main Pathfinding and Optimization Scripts:

1. **A_Ant_s.m**:
   - The main script that handles the execution of both the A* and ACO algorithms, including grid initialization, obstacle placement, and target point selection.

2. **Astar_s.m**:
   - Implements the A* algorithm. It calculates the optimal path between a start and a target point while avoiding obstacles using an open and closed list mechanism.

3. **ACATSP.m**:
   - Implements the Ant Colony Optimization (ACO) algorithm to solve the Traveling Salesman Problem (TSP) by finding the optimal sequence to visit multiple target points.

### Utility Functions:

1. **direction.m**:
   - Computes the direction of nodes relative to one another, used to calculate smooth paths between nodes.

2. **distance.m**:
   - Computes the Euclidean distance between two points, which is used by both A* and ACO algorithms for heuristic calculations.

3. **expand_array01.p**:
   - Handles the expansion of nodes in the A* algorithm, checking neighboring nodes to determine valid movements.

4. **insert_open.m**:
   - Populates the open list with nodes during the A* search process.

5. **min_fn.m**:
   - Selects the node with the minimum cost in the open list for expansion in the A* algorithm.

6. **MAP.m**:
   - Initializes the grid map with obstacles and sets up the grid structure for pathfinding.

### Path and Node Indexing Functions:

1. **node_index.m / node_index9.m**:
   - Provides functionality to retrieve the index of a node in the open or closed list.

2. **path_node_index.m**:
   - Retrieves the index of a node in the path list, helping trace the path back from the target to the start.

3. **Optimal_index.m**:
   - Finds the optimal node index in a given list to guide path generation.

### Angle and Direction Calculation:

1. **myangle.m / sn_angle.m**:
   - Calculates the angle between nodes or vectors, which helps in ensuring smooth transitions between path segments.

2. **toDegree.m / toRadian.m**:
   - Converts angles between degrees and radians, ensuring compatibility with various mathematical operations.

### Grid and Target Handling:

1. **MAX.m**:
   - Provides the grid map structure with static obstacles.

2. **Zuo_st.m**:
   - Sets up the initial start and target points on the grid.

3. **Target_node.m**:
   - Handles dynamic target node selection based on the proximity of dynamic obstacles.

## How to Use

1. **Run the main script** (`A_Ant_s.m`):
   - The user selects start and target points by clicking on the grid.
   - The system runs A* to find the shortest path and uses ACO to optimize the sequence of visiting multiple target points.

2. **Visualization**:
   - The optimized path is displayed, showing the route taken from the start to the target points while avoiding obstacles.

3. **Dynamic Adjustments**:
   - The system can dynamically adjust to new obstacles that appear during execution.

## Customization

- **ACO Parameters**: Modify `ACATSP.m` to adjust parameters like the number of ants, evaporation rate, and the number of iterations.
  
- **A* Parameters**: Adjust grid dimensions or obstacle density in `MAP.m` to simulate different environments.
  
- **Dynamic Obstacle Handling**: Modify `Target_node.m` to fine-tune the distance thresholds for dynamic obstacle avoidance.

## Example

1. Start by running the main script `A_Ant_s.m`.
2. Choose the start and target points on the grid.
3. The system will compute the optimal path, considering both static and dynamic obstacles, and plot the result.
![image](https://github.com/user-attachments/assets/8f00b0bd-445f-45d8-8ad6-5b8006d5dec1)
![image](https://github.com/user-attachments/assets/41024c32-328a-44fa-a8f1-bf4135cea0ba)
![image](https://github.com/user-attachments/assets/10594c0d-7ae5-4242-be81-76adcc1c2736)
![image](https://github.com/user-attachments/assets/69062352-8dea-44ee-b532-0d0286e8c566)
![image](https://github.com/user-attachments/assets/8cd994ad-62c9-4c32-b645-621c5b112f48)
