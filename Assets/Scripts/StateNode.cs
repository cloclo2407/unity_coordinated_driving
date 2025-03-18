using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.Numerics;
using Scripts.Map;
using UnityEditor;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using static AIP1TrafficCar; //Allows us to access static fields of AIP1TrafficCar
using static AIP2TrafficDrone;

public class StateNode : IComparable<StateNode> { 
    //Inherits from IComparable<StateNode> in order to be able to compare two StateNodes, see CompareTo() method below.
    //StateNode-objects are used to keep track of states/ nodes/ positions we visit while constructing a path
    public int myCarIndex;
    public static float cell_size;
    public Vector3 start_pos_global;
    public Vector3 goal_pos_global;
    public Vector3 world_position; //other way of saying global_position
    public float orientation; //Orientation around y-axis at this node, in degrees. Remember Unity uses left-handed coord. sys.
    public Vector3Int cell_position;
    public StateNode parent_node;
    public float cost_to_come;
    public float cost_to_go;                                           
    //public float turning_penalty;
    public float other_starts_and_goals_penalty; //Penalty for choosing positions close to other cars' goals
    public float close_to_obstacle_penalty; //High penalty to stay
    public float combined_cost;             //away from walls
    // Implementing a hashset (dictionary with only keys) for keeping track of all visited nodes by all runs of A* for all cars
    // This is intended to make A* choose less overlapping paths for different cars, used as reason for added cost (penalty).
    //public static HashSet<Vector3Int> used_waypoints = new HashSet<Vector3Int>();
    //public float close_to_used_wp_penalty; //add penalty for being close to used waypoint in other run of A*
    public float smooth_start_distance = 30f; //How far we let A* use smoother 30-degree turns in the start of a path

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
    public StateNode(Vector3 world_position, float orientation, Vector3 start_world_pos, Vector3 goal_world_pos, StateNode parent_node, MapManager mapManager, ObstacleMap obstacleMap, int myCarIndex)
    {
        this.start_pos_global = start_world_pos;
        this.goal_pos_global = goal_world_pos;
        //this.local_position = local_position;
        this.world_position = world_position;
        this.orientation = orientation;
        this.cell_position = obstacleMap.WorldToCell(world_position);
        this.parent_node = parent_node;
        calculateCostToCome(); //setting this.cost_to_come as length of the path leading up to this node
        // Vector3 has x,y,z-components of type float. Math.Pow() takes doubles and returns double. Math.Sqrt() takes double and returns double.
        // (float) converts the double returned by the Math functions to float.
        float euclidean_distance_to_goal = (float)( Math.Sqrt(  Math.Pow(goal_world_pos.x - this.world_position.x, 2d) + Math.Pow(goal_world_pos.z - this.world_position.z, 2d)  ) );
        //float weighted_euclidean_distance = euclidean_distance_to_goal * 0.2f; //If weight>1 A* prioritises sticking closer to goal, if weight<1 A* prioritises it less
        //float manhattan_distance_to_goal = Mathf.Abs(goal_world_pos.x - world_position.x) + Mathf.Abs(goal_world_pos.z - world_position.z);
        //float chebyshev_distance_to_goal = Mathf.Max(Mathf.Abs(goal_world_pos.x - world_position.x), Mathf.Abs(goal_world_pos.z - world_position.z));
        //float dijkstra_distance_to_goal = 0;
        this.cost_to_go = euclidean_distance_to_goal;
        
        //Calculate turning penalty based on orientation of this and parent_node
        /*
        if (parent_node == null)                              //Start node has no parent
        { this.turning_penalty = 0f; }
        else if (this.orientation != parent_node.orientation) //We have made a turn
        { this.turning_penalty = 1f; }
        else                                                  //We have not made a turn
        { this.turning_penalty = 0f; }
        */
        
        //For calculating close_to_used_wp_penalty I need to check the distance to closest used waypoint.
        // How can I, given a Vector3Int check which Vector3Int in my hashset is closest and retrieve that Vector3Int?
        // Sounds slightly computationally expensive to me :l
        
        //this.close_to_used_wp_penalty = (used_waypoints.Contains(cell_position) ? 10f : 0f); //If reused waypoint, cost+=10
        this.close_to_obstacle_penalty = calculateObstaclePenalty();
        this.other_starts_and_goals_penalty = calculateOtherStartsAndGoalsPenalty(mapManager);
        this.combined_cost = this.cost_to_come + this.cost_to_go + this.close_to_obstacle_penalty + this.other_starts_and_goals_penalty; //+ this.close_to_used_wp_penalty; //+ turning_penalty;
        this.myCarIndex = myCarIndex;
    }

    private float calculateOtherStartsAndGoalsPenalty(MapManager mapManager)
    {
        
        var all_start_pos = mapManager.startPositions;
        var all_goal_pos = mapManager.targetPositions;
        
        float close_to_other_start_penalty = 0f;
        float close_to_other_goal_penalty = 0f;

        for (int j = 0; j < all_start_pos.Count; j++)
        {
            float distance_to_start = Vector3.Distance(this.world_position, all_start_pos[j]);
            if ((distance_to_start < 25f) && all_start_pos[j] != this.start_pos_global) //if distance_to_start<5, and it's not our start 
            {
                if (distance_to_start == 0f) //Just to not risk raising division by zero error
                { close_to_other_start_penalty += 1000f; }
                else 
                {
                    //Increase penalty for every goal at distance <10 that is not our goal
                    close_to_other_start_penalty += (1/distance_to_start)*100f;
                }
            }
        }
        
        for (int k = 0; k < all_goal_pos.Count; k++)
        {
            float distance_to_goal = Vector3.Distance(this.world_position, all_goal_pos[k]);
            if ((distance_to_goal < 10f) && all_goal_pos[k] != this.goal_pos_global) //if distance_to_goal<10, and it's not our goal 
            {
                if (distance_to_goal == 0f) //Just to not risk raising division by zero error
                { close_to_other_goal_penalty += 1000f; }
                else 
                {
                    //Increase penalty for every goal at distance <10 that is not our goal
                    close_to_other_start_penalty += (1/distance_to_goal)*100f;
                }
            }
        }

        return close_to_other_start_penalty + close_to_other_goal_penalty;
    }
    
    private void calculateCostToCome()
    { //Calculates cost_to_come as the length of the path so far
        
        if (this.parent_node == null) //This is the root node/ starting node, cost to come here is 0.
        { this.cost_to_come = 0f; }
        
        else //This is not the root node
        {
            this.cost_to_come = parent_node.cost_to_come + Vector3.Distance(this.parent_node.world_position, this.world_position);
        }
    }
    
    private float calculateObstaclePenalty()
    {   // Calculates and sets close_to_obstacle_penalty for a new StateNode.
        // This takes into account obstacles at new positions and to the left of new positions
        // Being close to obstacles should be penalised, having obstacles on the left side should be penalised '
        // (we want to keep left side free and drive on the right)
        
        // Get all colliders within the sphere
        Collider[] hit_colliders = Physics.OverlapSphere(this.world_position, 5f, LayerMask.GetMask("Obstacle"));
        
        // Initialize variables to store the closest hit info
        float min_hit_distance = Mathf.Infinity;

        // Iterate through each collider to find the closest hit point
        foreach (Collider col in hit_colliders)
        {
            // Get the closest point on the collider to the origin.
            Vector3 hit_point = col.ClosestPoint(this.world_position);
            float hit_distance = Vector3.Distance(hit_point, this.world_position);

            if (hit_distance < min_hit_distance) min_hit_distance = hit_distance;
        }

        Vector3 forward = Quaternion.Euler(0, this.orientation, 0) * Vector3.forward;
        // Compute the left direction vector
        Vector3 leftDirection = Quaternion.Euler(0, -90, 0) * forward;

        // Compute the new position
        Vector3 onMyLeft = this.world_position + leftDirection * 4f;

        // Get all colliders within the sphere
        hit_colliders = Physics.OverlapSphere(onMyLeft, 6f, LayerMask.GetMask("Obstacle"));

        // Initialize variables to store the closest hit info
        float min_left_hit_distance = Mathf.Infinity;

        // Iterate through each collider to find the closest hit point
        foreach (Collider col in hit_colliders)
        {
            // Get the closest point on the collider to the origin.
            Vector3 hit_point = col.ClosestPoint(this.world_position);
            float hit_distance = Vector3.Distance(hit_point, this.world_position);

            if (hit_distance < min_left_hit_distance) min_left_hit_distance = hit_distance;
        }
        return (1/min_hit_distance)*10f + (1/min_left_hit_distance)*1000f; //The smaller the distance to closest obstacle, the higher the penalty cost
    }

    public List<StateNode> makeChildNodes(Dictionary<Vector3Int, StateNode> visited_nodes, PriorityQueue Q, 
        MapManager mapManager, ObstacleMap obstacleMap, float cellength, String vehicle)
    {
        // Returns list of childnodes that point to this, the parent node
        List<StateNode> childnodes_list = new List<StateNode>();

        //float stepsize = cell_scale.z; //6.43765f; //I made this line redundant by using cell_scale.z as stepsize. NOTE: Need cell_scale.z = cell_scale.x while running!
        float drone_turning_angle = 45f; //Turning angle in degrees.
        float car_average_turning_angle = 30f; //Average turning angle in degrees (average because I keep grid alignment and have to choose the cell that best fits 30 degree turns)
        
        // For the car I'm allowing the path to move forward (north), left or right diagonally a.k.a northwest and northeast.
        // For the drone I'm allowing straight movements forward, backward, left, right (north, south, west, east) as well as
        // northwest, northeast, southwest, southeast. Each movement is here defined by a new global position (Vector3,
        // unnormalized) and which new orientation to adopt at that position (float).
        // Note about the orientation: it's a float specifying rotation around the y-axis.
        List<Tuple<Vector3, float>> potential_movements = new List<Tuple<Vector3, float>>();

        //OK, so... I want every step that A* takes to cover the same amount of cells, cellsize. Every node is going to
        // have a rotation that is a multiple of 45f (turning_angle). When the rotation of currentnode is ...0,90,180,270...
        // a step forward will be cellength. Turning by 45f will make us move diagonally in relation to the grid, which
        // is a distance sqrt(cellength^2 + cellength^2) long to cover the same amount of cells. When currentnode has 
        // rotation ...45,135,225,315... a step forward will be diagonal in relation to the grid. It should then be a 
        // distance sqrt(cellength^2 + cellength^2). Turning +-45f will make the movement aligned with the grid and should 
        // therefore have a distance of cellength.
        
        if (vehicle == "car")
        {
            //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.Log("this.orientation: "+this.orientation);
            //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.Log("cost_to_come: "+this.cost_to_come);
            
            if (cost_to_come < smooth_start_distance) //Checking if limit of how far we've used 30-degree turns is reached
            {
                //float closest_multiple_of_30 = Mathf.Round(this.orientation / 30f) * 30f; //Finds closest multiple of 30 degrees to this.orientation
                //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.Log("closest_mult_30: "+closest_multiple_of_30);
                //float remainder = ((closest_multiple_of_30 % 90) + 90) % 90; // Ensure remainder is always in range [0, 90)
                float remainder = ((this.orientation % 90) + 90) % 90; // Ensure remainder is always in range [0, 90)
                //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.Log("remainder: "+remainder);
                //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.Log("-1f*Mathf.Atan(0.5f): "+(Mathf.Rad2Deg*(-1f*Mathf.Atan(0.5f))));
                //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.Log("2f*Mathf.Asin(Mathf.Sqrt(2f)/(2f*Mathf.Sqrt(5f))): "+ (Mathf.Rad2Deg*(2f*Mathf.Asin(Mathf.Sqrt(2f)/(2f*Mathf.Sqrt(5f))))));
            
                if (remainder == 0)
                {   // Reset orientation to multiple of 90 degrees when its closest multiple of 30 is a multiple of 90:
                    // With this code it should (based on my calculations on paper) round 90,1 degrees to 90.
                    //this.orientation = Mathf.Round(this.orientation);
                
                    // Angles -360,-270,-180,-90,0,90,180,270,360, north is aligned with grid
                
                    // move straight north by 2f*cellength:
                    Tuple<Vector3, float> north =
                    new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) * 2f*cellength
                        + this.world_position, this.orientation);
                
                    // move northwest diagonally by sqrt(cellength^2 + (2*cellength)^2) = sqrt(5*cellength^2) = sqrt(5)*cellength:
                    float nw_turning_angle = Mathf.Rad2Deg * (-1f*Mathf.Atan(0.5f));
                    Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + nw_turning_angle, 0) * Vector3.forward) * (Mathf.Sqrt(5f)*cellength)
                    + this.world_position, this.orientation - car_average_turning_angle);
                
                    // move northeast diagonally by sqrt(5)*cellength:
                    float ne_turning_angle = Mathf.Rad2Deg * (Mathf.Atan(0.5f));
                    Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + ne_turning_angle, 0) * Vector3.forward) * (Mathf.Sqrt(5f)*cellength)
                    + this.world_position, this.orientation + car_average_turning_angle);
                
                    potential_movements.Add(north);
                    potential_movements.Add(north_west);
                    potential_movements.Add(north_east);
                }
                else if (remainder == 30)
                {   //Angles -330,-240,-150,-60,30,120,210,300, north_west is aligned with grid
                
                    // move north diagonally by sqrt(cellength^2 + (2*cellength)^2) = sqrt(5*cellength^2) = sqrt(5)*cellength:
                    Tuple<Vector3, float> north = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) * (Mathf.Sqrt(5f)*cellength)
                    + this.world_position, this.orientation);
                
                    // move straight northwest by 2f*cellength:
                    float nw_turning_angle = Mathf.Rad2Deg * (-1f*Mathf.Atan(0.5f));
                    Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + nw_turning_angle, 0) * Vector3.forward) * 2f*cellength
                    + this.world_position, this.orientation - car_average_turning_angle);
                
                    // move northeast diagonally by sqrt(5)*cellength:
                    float ne_turning_angle = Mathf.Rad2Deg * (2f * Mathf.Asin( Mathf.Sqrt(2f)/ (2f*Mathf.Sqrt(5f)) ));
                    Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + ne_turning_angle, 0) * Vector3.forward) * (Mathf.Sqrt(5f)*cellength)
                    + this.world_position, this.orientation + car_average_turning_angle);
                
                    potential_movements.Add(north);
                    potential_movements.Add(north_west);
                    potential_movements.Add(north_east);
                }
                else if (remainder == 60)
                {
                    //Angles -300,-210,-120,-30,60,150,240,330, north_east is aligned with grid

                    // move north diagonally by sqrt(cellength^2 + (2*cellength)^2) = sqrt(5*cellength^2) = sqrt(5)*cellength:
                    Tuple<Vector3, float> north = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) * (Mathf.Sqrt(5f) * cellength)
                    + this.world_position, this.orientation);

                    // move northwest diagonally by sqrt(5)*cellength:
                    float nw_turning_angle = Mathf.Rad2Deg * (-2f * Mathf.Asin(Mathf.Sqrt(2f) / (2f * Mathf.Sqrt(5f))));
                    Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + nw_turning_angle, 0) * Vector3.forward) *
                    (Mathf.Sqrt(5f) * cellength)
                    + this.world_position, this.orientation - car_average_turning_angle);

                    // move straight northeast by 2f*cellength:
                    float ne_turning_angle = Mathf.Rad2Deg * (1f * Mathf.Atan(0.5f));
                    Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + ne_turning_angle, 0) * Vector3.forward) * 2f * cellength
                    + this.world_position, this.orientation + car_average_turning_angle);

                    potential_movements.Add(north);
                    potential_movements.Add(north_west);
                    potential_movements.Add(north_east);
                } 
            }
            else //Time to switch to 45-degree turns
            {
                if ( Mathf.Round(Vector3.Distance(parent_node.world_position, this.world_position)) 
                    == Mathf.Round(Mathf.Sqrt(5f)*cellength) || 
                    Mathf.Round(Vector3.Distance(parent_node.world_position, this.world_position))
                    == Mathf.Round(2f*cellength) )//If last move was 30-degrees, this will return true
                {
                    this.world_position = SnapToGridCenter(this.world_position); //Round position to closest cell-center
                    this.orientation = Mathf.Round(this.orientation / 45f) * 45f; //Round orientation to closest multiple of 45 degrees
                }
                
                if (this.orientation % 90f == 0f)
                {
                    //orientation = ...0,90,180,270...
                    // Rotation is aligned with the grid. Moving forwards will keep grid alignment.
                    // Moving to diagonal cell will require a step sqrt(cellength^2+cellength^2)

                    // move north by cellength:
                    Tuple<Vector3, float> north =
                    new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) * cellength
                                              + this.world_position, this.orientation);
                    // move northwest diagonally by sqrt(2*cellength^2):
                    Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - drone_turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    this.orientation - drone_turning_angle);
                    // move northeast diagonally by sqrt(2*cellength^2):
                    Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + drone_turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    this.orientation + drone_turning_angle);

                    potential_movements.Add(north);
                    potential_movements.Add(north_west);
                    potential_movements.Add(north_east);
                }
                else
                {   
                    //this.orientation is multiple of 45 degrees
                    // Rotation is not aligned with grid. Moving diagonally will realign orientation with grid.
                    // Moving forwards will require a step sqrt(cellength^2+cellength^2)

                    // move north by sqrt(2*cellength^2):
                    Tuple<Vector3, float> north = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    this.orientation);
                    // move northwest by cellength:
                    Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - drone_turning_angle, 0) * Vector3.forward) * cellength
                    + this.world_position, this.orientation - drone_turning_angle);
                    // move northeast by cellength:
                    Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + drone_turning_angle, 0) * Vector3.forward) * cellength
                    + this.world_position, this.orientation + drone_turning_angle);
                    
                    potential_movements.Add(north); 
                    potential_movements.Add(north_west);
                    potential_movements.Add(north_east);
                }
            }
        }
        else if (vehicle == "drone")
        {
            if (this.orientation % 90f == 0f)
            {
                //orientation = ...0,90,180,270...
                // Rotation is aligned with the grid. Moving forwards will keep grid alignment.
                // Moving to diagonal cell will require a step sqrt(cellength^2+cellength^2)

                // move north by cellength:
                Tuple<Vector3, float> north =
                    new Tuple<Vector3, float>((Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) * cellength
                                              + this.world_position, NormalizeAngle(this.orientation));
                // move northwest diagonally by sqrt(2*cellength^2):
                Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - drone_turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d))))
                    + this.world_position, NormalizeAngle(this.orientation - drone_turning_angle));
                // move northeast diagonally by sqrt(2*cellength^2):
                Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + drone_turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    NormalizeAngle(this.orientation + drone_turning_angle));
                // move south by cellength
                Tuple<Vector3, float> south =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation + 4 * drone_turning_angle, 0) * Vector3.forward) *
                        cellength + this.world_position, NormalizeAngle(this.orientation + 4 * drone_turning_angle));
                // move southwest diagonally by sqrt(2*cellength^2):
                Tuple<Vector3, float> south_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - 3 * drone_turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    NormalizeAngle(this.orientation - 3 * drone_turning_angle));
                // move southeast diagonally by sqrt(2*cellength^2):
                Tuple<Vector3, float> south_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + 3 * drone_turning_angle, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    NormalizeAngle(this.orientation + 3 * drone_turning_angle));
                // move west by cellength
                Tuple<Vector3, float> west =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation - 2 * drone_turning_angle, 0) * Vector3.forward) *
                        cellength + this.world_position, NormalizeAngle(this.orientation - 2 * drone_turning_angle));
                // move east by cellength
                Tuple<Vector3, float> east =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation + 2 * drone_turning_angle, 0) * Vector3.forward) *
                        cellength + this.world_position, NormalizeAngle(this.orientation + 2 * drone_turning_angle));

                potential_movements.Add(north);
                potential_movements.Add(north_west);
                potential_movements.Add(north_east);
                potential_movements.Add(south);
                potential_movements.Add(south_west);
                potential_movements.Add(south_east);
                potential_movements.Add(west);
                potential_movements.Add(east);
            }
            else
            {   
                //this.orientation is multiple of 45 degrees
                // Rotation is not aligned with grid. Moving diagonally will realign orientation with grid.
                // Moving forwards will require a step sqrt(cellength^2+cellength^2)

                // move north by sqrt(2*cellength^2):
                Tuple<Vector3, float> north = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation, 0) * Vector3.forward) *
                    (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                    NormalizeAngle(this.orientation));
                // move northwest by cellength:
                Tuple<Vector3, float> north_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - drone_turning_angle, 0) * Vector3.forward) * cellength
                    + this.world_position, NormalizeAngle(this.orientation - drone_turning_angle));
                // move northeast by cellength:
                Tuple<Vector3, float> north_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + drone_turning_angle, 0) * Vector3.forward) * cellength
                    + this.world_position, NormalizeAngle(this.orientation + drone_turning_angle));
                // move south by sqrt(2*cellength^2):
                Tuple<Vector3, float> south =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation + 4 * drone_turning_angle, 0) * Vector3.forward) *
                        (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                        NormalizeAngle(this.orientation + 4 * drone_turning_angle));
                // move southwest by cellength:
                Tuple<Vector3, float> south_west = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation - 3 * drone_turning_angle, 0) * Vector3.forward) * cellength
                    + this.world_position, NormalizeAngle(this.orientation - 3 * drone_turning_angle));
                // move southeast by cellength:
                Tuple<Vector3, float> south_east = new Tuple<Vector3, float>(
                    (Quaternion.Euler(0, this.orientation + 3 * drone_turning_angle, 0) * Vector3.forward) * cellength
                    + this.world_position, NormalizeAngle(this.orientation + 3 * drone_turning_angle));
                // move west by sqrt(2*cellength^2):
                Tuple<Vector3, float> west =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation - 2 * drone_turning_angle, 0) * Vector3.forward) *
                        (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                        NormalizeAngle(this.orientation - 2 * drone_turning_angle));
                // move east by sqrt(2*cellength^2):
                Tuple<Vector3, float> east =
                    new Tuple<Vector3, float>(
                        (Quaternion.Euler(0, this.orientation + 2 * drone_turning_angle, 0) * Vector3.forward) *
                        (float)(Math.Sqrt(2d * (Math.Pow((double)cellength, 2d)))) + this.world_position,
                        NormalizeAngle(this.orientation + 2 * drone_turning_angle));

                potential_movements.Add(north);
                potential_movements.Add(north_west);
                potential_movements.Add(north_east);
                potential_movements.Add(south);
                potential_movements.Add(south_west);
                potential_movements.Add(south_east);
                potential_movements.Add(west);
                potential_movements.Add(east);
            }
        }

        //Make sure new nodes are not in same pos and opposite direction of other cars' paths:
        var non_conflicting_movements = checkForConflictingPaths(this.world_position, potential_movements, vehicle);
        //Make sure new nodes give an obstacle free path:
        var valid_movements = checkForObstacles(non_conflicting_movements);

        foreach (var valid_movement in valid_movements)
        {
            var new_position = valid_movement.Item1;
            var new_orientation = valid_movement.Item2;
            StateNode new_node = new StateNode(new_position, new_orientation, this.start_pos_global, this.goal_pos_global, this, mapManager, obstacleMap, myCarIndex);
            
            //I want to draw out all positions we visit to get a feel of how A* explores the space:
            //if (AIP1TrafficCar.crazyCarIndex == myCarIndex) Debug.DrawLine(this.world_position+Vector3.up*0.1f, new_position+Vector3.up*0.1f, Color.green, 1000f);
            
            //Have we seen this potential node/ cell before? Let's check Q first and then visited_nodes. Maybe should rewire path for a shorter one!
            StateNode prev_node_in_Q = null;
            StateNode prev_node_in_visited_nodes = null;
            
            foreach (StateNode sn in Q) { if (sn.cell_position == new_node.cell_position)
            { prev_node_in_Q = sn;
                break; } }

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
                    prev_node_in_Q.combined_cost = prev_node_in_Q.cost_to_come + 
                                                   prev_node_in_Q.cost_to_go +
                                                   prev_node_in_Q.close_to_obstacle_penalty + 
                                                   prev_node_in_Q.other_starts_and_goals_penalty; //+ prev_node_in_Q.close_to_used_wp_penalty; // + prev_node_in_Q.turning_penalty;
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
                                                               prev_node_in_visited_nodes.close_to_obstacle_penalty +
                                                               prev_node_in_visited_nodes.other_starts_and_goals_penalty; // + prev_node_in_visited_nodes.close_to_used_wp_penalty; // + prev_node_in_visited_nodes.turning_penalty;
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

    private static List<Tuple<Vector3, float>> checkForConflictingPaths(Vector3 current_position, List<Tuple<Vector3, float>> potential_movements, String vehicle)
    {   //This method uses the Dictionary globalPathRegistry to lookup positions that have been used in other cars' paths.
        //The orientations used at a previously used position are stored in a Hashset as the value to the key which is the global position.
        //If we explore a new position in A* that has previously been used, we have to ensure that it was used with orientations 
        //that were not the opposite of the new orientation that we would adopt at the new position in this new node. If that were the case, 
        //the new node is discarded in order to avoid overlapping paths in opposite directions (which can cause head-on collisions).
        //Dictionaries are implemented as Hashtables. Searches in Hashtables and Hashsets have time complexity O(1) on average, which means 
        //this shouldn't add a lot of time to the planning phase - hopefully.
        
        List<Tuple<Vector3, float>> non_conflicting_movements = new List<Tuple<Vector3, float>>();
        Dictionary<Vector3, HashSet<StateNode>> global_path_registry = new Dictionary<Vector3, HashSet<StateNode>>();
        
        if (vehicle == "car")
        {
            global_path_registry = AIP1TrafficCar.globalPathRegistry;
            //Debug.Log("Vehicle is car, global_path_registry.Count: " + global_path_registry.Count());
            for (int i = 0; i < potential_movements.Count; i++)
            {
                bool non_conflicting_movement = true;
                var potential_new_position = potential_movements[i].Item1; //Access first element in tuple by calling .Item1
                if (global_path_registry.ContainsKey(potential_new_position))
                {
                    //Debug.Log("potential_new_pos is in globalPathReg");
                    float potential_new_orientation = potential_movements[i].Item2;
                    float[] opposite_orientations = {potential_new_orientation - 180f, potential_new_orientation + 180f};
                    HashSet<StateNode> existing_nodes = global_path_registry[potential_new_position];
                    foreach (float opposite_orientation in opposite_orientations)
                    {
                        if (non_conflicting_movement == false) break; //Found a conflicting movement already in loop below
                        foreach (StateNode existing_node in existing_nodes)
                        {
                            //Debug.Log("Checking node at potential_new_pos...");
                            if (existing_node.orientation == opposite_orientation) //If any node in hashset of nodes at this world_pos contains opposite orientation
                            {
                                //We have a conflicting movement, do not add this potential movement as new node, to queue
                                //Debug.Log("Conflicting movement!");
                                non_conflicting_movement = false;
                                break;
                            }
                        }
                    }
                }
                //Add potential_movement[i] if it does not lead to a new node in opposite direction of existing node in same position
                if (non_conflicting_movement == true) non_conflicting_movements.Add(potential_movements[i]);
            }
        }
        
        else if (vehicle == "drone")
        { 
            global_path_registry = AIP2TrafficDrone.globalPathRegistry;
            //Debug.Log("Vehicle is drone, global_path_registry.Count: " + global_path_registry.Count());
            
            
            for (int j = 0; j < potential_movements.Count; j++)
            {
                bool non_conflicting_movement = true;
                Vector3 potential_new_position = potential_movements[j].Item1; //Access first element in tuple by calling .Item1
                
                /*
                foreach (var key in global_path_registry.Keys) {
                    if (Vector3.Distance(key, potential_new_position) < 0.2f) {
                        Debug.Log("Potential match found: " + key + " ≈ " + potential_new_position);
                    }
                }*/
                
                
                if (global_path_registry.ContainsKey(Rounded(potential_new_position))) //Is new_pos in globalPathReg?
                {
                    //Debug.Log("potential_new_pos is in globalPathReg");
                    if (global_path_registry.ContainsKey(Rounded(current_position))) //Is current_pos in globalPathReg?
                    {
                        //Debug.Log("current_pos is in globalPathReg");
                        HashSet<StateNode> existing_nodes_at_current_position = global_path_registry[Rounded(current_position)];
                        foreach (StateNode existing_node in existing_nodes_at_current_position)
                        {
                            //Debug.Log("Checking node at current_pos...");
                            if (existing_node.parent_node != null)
                            {
                                if (Rounded(existing_node.parent_node.world_position) == Rounded(potential_new_position)) //Is there a parent to node at current_pos, at new_pos?
                                {
                                    //We have a node at new_pos with child at current_pos already, so we shouldn't move from
                                    // current_pos to new_pos as that would become an overlapping path to an existing path
                                    // in the opposite direction. These things cause gridlocks, so we need to filter them out
                                    //Debug.Log("Conflicting movement!");
                                    non_conflicting_movement = false;
                                    break;
                                }
                            }
                        }
                    }
                }
                //Add potential_movement[i] if it does not lead to a new node in opposite direction of existing node in same position
                if (non_conflicting_movement == true) non_conflicting_movements.Add(potential_movements[j]);
            }
        }
        
        return non_conflicting_movements;
    }
    
    private static Vector3 Rounded(Vector3 v) {
        return new Vector3(
            Mathf.Round(v.x * 1000) / 1000,  // Round to 3 decimal places
            Mathf.Round(v.y * 1000) / 1000,
            Mathf.Round(v.z * 1000) / 1000
        );
    }
    
    private static float NormalizeAngle(float angle_degrees)
    {//Method forces angles (in degrees) into the interval [-180,180]
        float angle_degrees_normalized = ((angle_degrees + 180f) % 360f + 360f) % 360f - 180f;
        return angle_degrees_normalized;
    }

    private List<Tuple<Vector3, float>> checkForObstacles(List<Tuple<Vector3, float>> potential_movements)
    {   //This method goes through every potential movement and sweeps a Sphere from this.world_pos to potential_new_pos
        //The sphere has radius 2.5f which just about covers the car, and ensures that chosen waypoints and the paths 
        //between them are free of obstacles. It also discards new positions too close to other cars' goal positions.
        List<Tuple<Vector3, float>> valid_movements = new List<Tuple<Vector3, float>>();

        for (int i = 0; i < potential_movements.Count; i++)
        {
            bool valid_movement = true;
            var potential_new_position = potential_movements[i].Item1; //Access first element in tuple by calling .Item1
            Vector3 direction = (potential_new_position - this.world_position).normalized; //Normalize heading to get direction vector
            float distance = Vector3.Distance(potential_new_position, this.world_position);
            
            RaycastHit hit; //Compiler gets mad at me if I don't claim the hit-object
            if (Physics.SphereCast(this.world_position, 2.5f, direction, out hit, distance, 
                    LayerMask.GetMask("Obstacle")) == true) { valid_movement = false; }
            
            if (valid_movement == true) valid_movements.Add(potential_movements[i]);
        }
        //Debug.Log("valid_movements: "+valid_movements.Count);
        return valid_movements;
    }
    
    // Function to round position coordinates to the center of the cell
    Vector3 SnapToGridCenter(Vector3 position)
    {
        return new Vector3(
            Mathf.Round(position.x / cell_size) * cell_size + cell_size / 2f,
            position.y, // Keep the original Y value
            Mathf.Round(position.z / cell_size) * cell_size + cell_size / 2f
        );
    }
    
    public void fillPaths(List<StateNode> path_of_nodes, List<Vector3> path_of_points) 

    {// Takes path and fills it with this node and all of its parents        
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