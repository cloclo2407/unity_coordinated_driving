using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Vehicle;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.UIElements;

public class StateNode : IComparable<StateNode> { 
    //Inherits from IComparable<StateNode> in order to be able to compare two StateNodes, see CompareTo() method below.
    //StateNode-objects are used to keep track of states/ nodes/ positions we visit while constructing a path
    public Vector3 world_position; //other way of saying global_position
    public float orientation; //Orientation around y-axis at this node, in degrees. Remember Unity uses left-handed coord. sys.
    public Vector3Int cell_position;
    public StateNode parent_node;
    public float cost_to_come;
    public float cost_to_go;                                           
    //public float turning_penalty;                                   
    public float close_to_obstacle_penalty; //High penalty to stay
    public float combined_cost;             //away from walls

    public int CompareTo(StateNode other)   //Call to use this method may look like: current_node.CompareTo(child_node)
    {                                                           //returns -1 if this.combined_cost < other.combined_cost
        return combined_cost.CompareTo(other.combined_cost);    //returns 0 if this.combined_cost = other.combined_cost
    }                                                           //returns 1 if this.combined_cost > other.combined_cost

    /*
    public override bool Equals(object obj) //Lets us compare StateNodes by their world_position, used by HashSet visited_nodes in CarAI.cs
    {
        if (obj is StateNode other)
        { return this.world_position == other.world_position; } //If argument object is StateNode type and they have the same world_position, return true
        return false; //Otherwise return false
    }
    
    public override int GetHashCode() //Lets the hashcode of HashSet<StateNode> objects be dependent on the world position of a StateNode node
    { return HashCode.Combine(world_position.x, world_position.y, world_position.z); } //Nodes at same pos will have same hashcode
    */
    
    //Constructor method:
    public StateNode(Vector3 world_position, float orientation, StateNode parent_node, MapManager mapManager, ObstacleMap obstacleMap)
    {
        //this.local_position = local_position;
        this.world_position = world_position;
        this.orientation = orientation;
        this.cell_position = obstacleMap.WorldToCell(world_position);
        this.parent_node = parent_node;
        this.cost_to_come = calculateCostToCome(); //setting cost_to_come = number of parent nodes

        var goal_world_pos = mapManager.GetGlobalGoalPosition();
        // Vector3 has x,y,z-components of type float. Math.Pow() takes doubles and returns double. Math.Sqrt() takes double and returns double.
        // (float) converts the double returned by the Math functions to float.
        float euclidean_distance_to_goal = (float)( Math.Sqrt(  Math.Pow(goal_world_pos.x - this.world_position.x, 2d) + Math.Pow(goal_world_pos.z - this.world_position.z, 2d)  ) );
        float weighted_euclidean_distance = euclidean_distance_to_goal * 0.2f; //If weight>1 A* prioritises sticking closer to goal, if weight<1 A* prioritises it less
        //float manhattan_distance_to_goal = Mathf.Abs(goal_world_pos.x - world_position.x) + Mathf.Abs(goal_world_pos.z - world_position.z);
        //float chebyshev_distance_to_goal = Mathf.Max(Mathf.Abs(goal_world_pos.x - world_position.x), Mathf.Abs(goal_world_pos.z - world_position.z));
        //float dijkstra_distance_to_goal = 0;
        this.cost_to_go = weighted_euclidean_distance;
        
        //Calculate turning penalty based on orientation of this and parent_node
        /*
        if (parent_node == null)                              //Start node has no parent
        { this.turning_penalty = 0f; }
        else if (this.orientation != parent_node.orientation) //We have made a turn
        { this.turning_penalty = 1f; }
        else                                                  //We have not made a turn
        { this.turning_penalty = 0f; }
        */
        
        this.close_to_obstacle_penalty = calculatePenalty(obstacleMap);
        this.combined_cost = this.cost_to_come + this.cost_to_go + this.close_to_obstacle_penalty; //+ turning_penalty;
    }

    private float calculateCostToCome()
    { //Returns cost_to_come as the number of parent nodes of this
        int parentcounter = 0;
        StateNode current = this;
        while (current.parent_node != null)
        {
            parentcounter++;
            current = current.parent_node; //point current at parent to check the parent of parent
        }
        return cost_to_come = parentcounter;
    }
    
    private float calculatePenalty(ObstacleMap obstacleMap)
    {   // Uses checkCellListForObstacles to see how many obstacles are at a distance of one cell away from this.cell_position
        // At 1 cell away from this cell, we have 8 cells
        List<Vector3Int> cells_one_cell_away = new List<Vector3Int>() {
            new Vector3Int(-1, 0, 0)+this.cell_position, //left of this.cell_position
            new Vector3Int(1, 0, 0)+this.cell_position,  //right of this.cell_position
            new Vector3Int(0, 0, -1)+this.cell_position, //below this.cell_position
            new Vector3Int(0, 0, 1)+this.cell_position,  //above this.cell_position
            
            new Vector3Int(1, 0, 1)+this.cell_position,  //northeast of this.cell_position
            new Vector3Int(-1, 0, 1)+this.cell_position, //northwest of this.cell_position
            new Vector3Int(1, 0, -1)+this.cell_position, //southeast of this.cell_position
            new Vector3Int(-1, 0, -1)+this.cell_position //southwest of this.cell_position
        };
        var free_cells = checkCellListForObstacles(cells_one_cell_away, obstacleMap);
        var number_of_obstacles_one_cell_away = cells_one_cell_away.Count - free_cells.Count;
        float penalty_multiplier = 100f;
        
        // TODO Check for
        // TODO obstacles two
        // TODO cells away?
        
        return number_of_obstacles_one_cell_away * penalty_multiplier;
    }
    
    private List<Vector3Int> checkCellListForObstacles(List<Vector3Int> cellList, ObstacleMap obstacleMap)
    {
        List<Vector3Int> unoccupied_cells = new List<Vector3Int>();

        foreach (Vector3Int cell in cellList) {
            var obstacles_in_cell = obstacleMap.GetObstaclesPerCell(cell);
            if (obstacles_in_cell.Count == 0) // if obstacle in cell, discard potential_headings[i] as valid heading by doing nothing 
            { unoccupied_cells.Add(cell); }
        }
        return unoccupied_cells;
    }

    public List<StateNode> makeChildNodes(Dictionary<Vector3Int, StateNode> visited_nodes, PriorityQueue Q, MapManager mapManager, ObstacleMap obstacleMap, float cellength, String vehicle)
    { // Returns list of childnodes that point to this, the parent node
        List<StateNode> childnodes_list = new List<StateNode>();

        //float stepsize = cell_scale.z; //6.43765f; //I made this line redundant by using cell_scale.z as stepsize. NOTE: Need cell_scale.z = cell_scale.x while running!
        float turning_angle = 45f; //Turning angle in degrees.
        
        // For the car I'm allowing the path to move forward (north), left or right diagonally a.k.a northwest and northeast.
        // For the drone I'm allowing straight movements forward, backward, left, right (north, south, west, east) as well as
        // northwest, northeast, southwest, southeast. Each movement is here defined by a tuple of movement to a
        // new position via a heading (Vector3, unnormalized) and which new orientation to adopt at that position (float).
        // Note about the orientation: it's a float specifying rotation around the y-axis.
        List<Tuple<Vector3, float>> potential_movements = new List<Tuple<Vector3, float>>();
        
        //OK, so... I want every step that A* takes to cover the same amount of cells, cellsize. Every node is going to
        // have a rotation that is a multiple of 45f (turning_angle). When the rotation of currentnode is ...0,90,180,270...
        // a step forward will be cellength. Turning by 45f will make us move diagonally in relation to the grid, which
        // is a distance sqrt(cellength^2 + cellength^2) long to cover the same amount of cells. When currentnode has 
        // rotation ...45,135,225,315... a step forward will be diagonal in relation to the grid. It should then be a 
        // distance sqrt(cellength^2 + cellength^2). Turning +-45f will make the movement aligned with the grid and should 
        // therefore have a distance of cellength.
        
        if (this.orientation % 90 == 0) 
        {   //orientation = ...0,90,180,270...
            // Rotation is aligned with the grid. Moving forwards will keep grid alignment.
            // Moving to diagonal cell will require a step sqrt(cellength^2+cellength^2)
            
            // move north by cellength:
            Tuple<Vector3, float> north =
                new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) * cellength,
                this.orientation);
            // move northwest diagonally by sqrt(2*cellength^2):
            Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                (Quaternion.Euler(0, this.orientation - turning_angle, 0) * Vector3.forward) *
                (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                this.orientation - turning_angle);
            // move northeast diagonally by sqrt(2*cellength^2):
            Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                (Quaternion.Euler(0, this.orientation + turning_angle, 0) * Vector3.forward) *
                (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                this.orientation + turning_angle);
            potential_movements.Add(north);
            potential_movements.Add(north_west);
            potential_movements.Add(north_east);

            if (vehicle == "drone") //The path for the drone has 5 more movement options
            { 
                // move south by cellength
                Tuple<Vector3, float> south =
                    new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation + 4*turning_angle, 0) * Vector3.forward) * cellength,
                        this.orientation + 4*turning_angle);
                // move southwest diagonally by sqrt(2*cellength^2):
                Tuple<Vector3, float> south_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - 3*turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                    this.orientation - 3*turning_angle);
                // move southeast diagonally by sqrt(2*cellength^2):
                Tuple<Vector3, float> south_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + 3*turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                    this.orientation + 3*turning_angle);
                // move west by cellength
                Tuple<Vector3, float> west =
                    new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation - 2*turning_angle, 0) * Vector3.forward) * cellength,
                        this.orientation - 2*turning_angle);
                // move east by cellength
                Tuple<Vector3, float> east =
                    new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation + 2*turning_angle, 0) * Vector3.forward) * cellength,
                        this.orientation + 2*turning_angle);
                potential_movements.Add(south);
                potential_movements.Add(south_west);
                potential_movements.Add(south_east);
                potential_movements.Add(west);
                potential_movements.Add(east);
            }
        }
        else { //this.orientation is multiple of 45 degrees
            // Rotation is not aligned with grid. Moving diagonally will realign orientation with grid.
            // Moving forwards will require a step sqrt(cellength^2+cellength^2)
                
            // move north by sqrt(2*cellength^2):
            Tuple<Vector3, float> north = new Tuple<Vector3, float>(
                (Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) *
                (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                this.orientation);
            // move northwest by cellength:
            Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                (Quaternion.Euler(0, this.orientation - turning_angle, 0) * Vector3.forward) * cellength, 
                this.orientation - turning_angle);
            // move northeast by cellength:
            Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                (Quaternion.Euler(0, this.orientation + turning_angle, 0) * Vector3.forward) * cellength,
                this.orientation + turning_angle);
            potential_movements.Add(north);
            potential_movements.Add(north_west);
            potential_movements.Add(north_east); 
            
            if (vehicle == "drone") //The path for the drone has 5 more movement options
            { 
                // move south by sqrt(2*cellength^2):
                Tuple<Vector3, float> south =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation + 4*turning_angle, 0) * Vector3.forward) * 
                        (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                        this.orientation + 4*turning_angle);
                // move southwest by cellength:
                Tuple<Vector3, float> south_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - 3*turning_angle, 0) * Vector3.forward) * cellength,
                    this.orientation - 3*turning_angle);
                // move southeast by cellength:
                Tuple<Vector3, float> south_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + 3*turning_angle, 0) * Vector3.forward) * cellength,
                    this.orientation + 3*turning_angle);
                // move west by sqrt(2*cellength^2):
                Tuple<Vector3, float> west =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation - 2*turning_angle, 0) * Vector3.forward) *
                        (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                        this.orientation - 2*turning_angle);
                // move east by sqrt(2*cellength^2):
                Tuple<Vector3, float> east =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation + 2*turning_angle, 0) * Vector3.forward) *
                        (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))),
                        this.orientation + 2*turning_angle);
                potential_movements.Add(south);
                potential_movements.Add(south_west);
                potential_movements.Add(south_east);
                potential_movements.Add(west);
                potential_movements.Add(east);
            }
        }
        
        var valid_movements = this.checkForObstacles(potential_movements, obstacleMap);

        foreach (var valid_movement in valid_movements)
        {
            var new_position = this.world_position + valid_movement.Item1;
            var new_orientation = valid_movement.Item2;
            StateNode new_node = new StateNode(new_position, new_orientation, this, mapManager, obstacleMap);
            
            //I want to draw out all positions we visit to get a feel of how A* explores the space:
            //Debug.DrawLine(this.world_position, new_position, Color.green, 1000f);
            
            //Have we seen this potential node/ cell before? Let's check Q first and then visited_nodes. Maybe should rewire path for a shorter one!
            StateNode prev_node_in_Q = null;
            StateNode prev_node_in_visited_nodes = null;
            
            foreach (StateNode sn in Q) { if (sn.cell_position == new_node.cell_position)
            { prev_node_in_Q = sn;
                break; } }

            /* foreach (StateNode s_n in visited_nodes) { if (s_n.cell_position == new_node.cell_position) 
            { prev_node_in_visited_nodes = s_n; 
                break; } } */

            if (visited_nodes.TryGetValue(new_node.cell_position, out StateNode existing_node)) //TryGetValue returns true if value exists at given key, false otherwise
            { prev_node_in_visited_nodes = existing_node; }
            
            //Debug.Log("prev_node_in_Q: "+(prev_node_in_Q != null)+", prev_node_in_visited_nodes: "+(prev_node_in_visited_nodes != null));

            //If new node is in Q and lower cost, we need to change it (update parent, cost...) (Q is sorted after this method returns)
            if (prev_node_in_Q != null)
            {
                //Check if new cost to come is lower than old one, for the same node/ cell
                if (prev_node_in_Q.cost_to_come > new_node.cost_to_come)
                {   //If true, update previously found node with current_node as parent, new cost_to_come, combined_cost
                    prev_node_in_Q.world_position = new_node.world_position;
                    prev_node_in_Q.orientation = new_node.orientation;
                    prev_node_in_Q.parent_node = this;
                    prev_node_in_Q.cost_to_come = new_node.cost_to_come;
                    prev_node_in_Q.cost_to_go = new_node.cost_to_go;
                    //prev_node_in_Q.turning_penalty = new_node.turning_penalty;
                    prev_node_in_Q.combined_cost = prev_node_in_Q.cost_to_come + prev_node_in_Q.cost_to_go +
                                                   prev_node_in_Q.close_to_obstacle_penalty; // + prev_node_in_Q.turning_penalty;
                }
            }
            
            //If new node is in visited_nodes and lower cost, we need to add to Q (Q is sorted after this method returns)
            else if (prev_node_in_visited_nodes != null)
            {
                //Check if new cost to come is lower than old one, for the same node/ cell
                if (prev_node_in_visited_nodes.cost_to_come > new_node.cost_to_come)
                {   //If true, update previously found node with current_node as parent, new cost_to_come, combined_cost
                    prev_node_in_visited_nodes.world_position = new_node.world_position;
                    prev_node_in_visited_nodes.orientation = new_node.orientation;
                    prev_node_in_visited_nodes.parent_node = this;
                    prev_node_in_visited_nodes.cost_to_come = new_node.cost_to_come;
                    prev_node_in_visited_nodes.cost_to_go = new_node.cost_to_go;
                    //prev_node_in_visited_nodes.turning_penalty = new_node.turning_penalty;
                    prev_node_in_visited_nodes.combined_cost = prev_node_in_visited_nodes.cost_to_come +
                                                               prev_node_in_visited_nodes.cost_to_go +
                                                               prev_node_in_visited_nodes.close_to_obstacle_penalty; // + prev_node_in_visited_nodes.turning_penalty;
                    //Q.Add(prev_node_in_visited_nodes);
                    Q.Enqueue(prev_node_in_visited_nodes); 
                    // Since we are adding the node back to Q, we should remove it from visited_nodes dict so that it can be added to the dict
                    // in CarAI.cs when expanding the popped node from Q. It's either this or add a check with .ContainsKey() over there 
                    // to not try to add something with the same key (cell_position) as a key-value pair that already exist in the dict.
                    visited_nodes.Remove(prev_node_in_visited_nodes.cell_position);
                }
            }
            
            else // else we should add this as a potential node to visit in queue.
            {    // new_node is added to visited_nodes after it has been taken out of Q and processed, in Start().
                childnodes_list.Add(new_node);
            }
        }
        
        //Debug.Log($"Number of children in childnodes_list: {childnodes_list.Count}");
        //Debug.Log($"Number of children in visited_nodes: {visited_nodes.Count}");
        return childnodes_list;
    }

    private List<Tuple<Vector3, float>> checkForObstacles(List<Tuple<Vector3, float>> potential_movements, ObstacleMap obstacleMap)
    {
        List<Tuple<Vector3, float>> valid_movements = new List<Tuple<Vector3, float>>();

        for (int i = 0; i < potential_movements.Count; i++)
        {
            var potential_new_position = this.world_position + potential_movements[i].Item1; //Access first element in tuple by calling .Item1
            var potential_new_cell = obstacleMap.WorldToCell(potential_new_position);
            var obstacles_in_new_cell = obstacleMap.GetObstaclesPerCell(potential_new_cell);
            
            //Debug.Log("Checking for obstacles at position "+potential_new_position+" and orientation "+potential_movements[i].Item2);

            if (obstacles_in_new_cell.Count > 0) // if obstacle in cell, discard potential_headings[i] as valid heading by doing nothing 
            { //Debug.Log("Obstacle found at position, discarding potential movement");
              continue; }
            
            // If no obstacle in new potential cell, check in between cells:
            Vector3 direction = potential_movements[i].Item1.normalized; //Normalize heading to get direction vector
            float distance = Vector3.Distance(this.world_position, potential_new_position);
            if (Physics.Raycast(this.world_position, direction, distance) == false)
            {   //No obstacle in between cells, also no obstacle at new cell, so we add this heading as a valid heading
                valid_movements.Add(potential_movements[i]);
            }
            else
            {
                //Debug.Log("Obstacle found via raycast, discarding potential movement");
            }
        }
        //Debug.Log("valid_movements: "+valid_movements.Count);
        return valid_movements;
    }
    
    public void fillPaths(List<StateNode> path_of_nodes, List<Vector3> path_of_points) 
    {// Returns path containing this node and all of its parents
        Debug.Log("Generating path...");
        
        path_of_nodes.Add(this);
        path_of_points.Add(this.world_position);
        
        StateNode current = this;
        
        while (current.parent_node != null)
        {
            current = current.parent_node; //point current at parent to check the parent of parent
            path_of_nodes.Add(current);
            path_of_points.Add(current.world_position);
        }
        path_of_nodes.Reverse(); //The path is filled from goal to start, so we have to reverse it to make it traversable
        path_of_points.Reverse();
    }
    
}