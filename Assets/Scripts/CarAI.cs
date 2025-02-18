using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.Numerics;
using Scripts.Vehicle;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using OpenCover.Framework.Model;
using Unity.Mathematics;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.UIElements;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;


[RequireComponent(typeof(CarController))]
public class CarAI_EDIT : MonoBehaviour
{
    // An example class, containing code snippets demonstrating how to do different things in the environment.

    private CarController m_Car; // the car controller we want to use
    private MapManager mapManager;
    private ObstacleMap obstacleMap;
    private BoxCollider carCollider;

    //public GameObject lookAheadSphere;
    
    private List<StateNode> path_of_nodes = new List<StateNode>();
    private List<Vector3> path_of_points = new List<Vector3>();
    private List<Vector3> smooth_path_of_points = new List<Vector3>();

    //For driving:
    private float k_p = 12; // 2f; // 2f;
    private float k_d = 0.05f; // 1f; // 0.5f;
    private float waypoint_margin = 3f; //Math.Clamp(my_rigidbody.linearVelocity.magnitude, 5f, 15f); //6.5f; //Serves as a means of checking if we're close enough to goal/ next waypoint
    private float speed_limit = 3.5f;
    private bool car_is_perpendicular = false;
    private float max_scan_distance = 7.5f; // Testing a variable scan distance
    //private bool obstacles_close = false;
    private List<Vector3> raycast_hit_positions = new List<Vector3>();
    private float obstacle_avoiding_steering = 0f; //Addition to steering input calculated via raycasts of environment
    //private float obstacle_avoiding_accel = 0f;    //in order to steer away from obstacles, but remain on path
    private float obstacle_avoiding_steering_limit = 40f; //TODO
    //private float obstacle_avoiding_accel_limit = 0f; //TODO
    //private bool open_area = false; //Sets parameters for driving in open areas, like TerrainA. TODO
    //private bool cluttered_area = true; //Sets parameters for driving in cluttered areas, like TerrainD. TODO
    
    private float steering; //Accessible in Inspector due to "public"
    private float accel;
    private float footbrake; //UNUSED?
    private float handbrake;
    
    Rigidbody my_rigidbody;
    
    private void Start()
    {
        Debug.Log("CarAI_EDIT: Start");
        
        // Get the car collider
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        // Get the car controller
        m_Car = GetComponent<CarController>();
        // Get the car's rigidbody
        my_rigidbody = GetComponent<Rigidbody>();
        // Initialize mapManager and obstacleMap
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;
        Vector3 cell_scale = Vector3.one * 2.6f;
        obstacleMap = ObstacleMap.Initialize(mapManager, new List<GameObject>(), cell_scale);
        obstacleMap.margin = Vector3.one * 1; // (Changing cell margins)
        
        ////////////////////////// Plan your path here
        Vector3 start_pos_global = mapManager.GetGlobalStartPosition();
        Vector3 goal_pos_global = mapManager.GetGlobalGoalPosition();
        
        //List<StateNode> Q = new List<StateNode>(); //Q is the queue holding the states we want to visit next
        PriorityQueue Q = new PriorityQueue();
        //List<StateNode> visited_nodes = new List<StateNode>(); //A list of visited cells
        Dictionary<Vector3Int, StateNode> visited_nodes = new Dictionary<Vector3Int, StateNode>();
        StateNode start_node = new StateNode(start_pos_global, 0f, null, mapManager, obstacleMap);
        //Q.Add(start_node);
        Q.Enqueue(start_node);
        
        while (Q.Count != 0) 
        {
            //var current_node = Q[0];
            //Q.RemoveAt(0); //Remove current node from list
            var current_node = Q.Dequeue();
            visited_nodes.Add(current_node.cell_position, current_node);
            
            if (Vector3.Distance(current_node.world_position, goal_pos_global) <= this.waypoint_margin) //We have reached the goal, time to create the path
            { 
                current_node.fillPaths(this.path_of_nodes, this.path_of_points); 
                //Now the path_of_nodes and path_of_points are ready to be executed. We can either use StateNodes or Vector3s in the controller.
                break; // BREAK OUT OF WHILE LOOP, WE'RE DONE HERE
            }
            //else we keep looking:
            List<StateNode> new_nodes = current_node.makeChildNodes(visited_nodes, Q, mapManager, obstacleMap, cell_scale.z, "car");
            //foreach (StateNode n in new_nodes) { Q.Add(n); } //Add all new nodes to the queue
            foreach (StateNode n in new_nodes) { Q.Enqueue(n); } //Add all new nodes to the queue
            
            // Sort the queue with lambda expression and List<T>.Sort() in ascending order according to combined_cost
            //Q.Sort((nodeA, nodeB) => (nodeA.combined_cost).CompareTo(nodeB.combined_cost));
            // This is a slow way of doing things. I should insert each new node at their correct position in Q for fastest result
            // TerrainB takes little over 40 seconds to find a path wiht the Q.Sort() and visited_nodes being a list of nodes.
            // For improvement, I'll insert nodes at the right index in Q instead of sorting, and visited_nodes can be made into
            // something that is easier to search through as the number of nodes gets large.
            // UPDATE: I've added a PriorityQueue-class as a binary min-heap in file PriorityQueue.cs. This reduces load time on
            // Terrain B to 37 seconds. About a 5s reduction I'd say. I'm now going to replace visited_nodes as List<StateNode>
            // with HashSet<StateNode>. NVM! I replaced it with a Dictionary (which is basically a HashTable in C#) and 
            // damn, I should've done this sooner because that cut down the loading time to just under 10s on TerrainB! 27 seconds shaved off!
            // Moral of the story: if you need to keep track of objects, but you don't need them to be ordered, use a dictionary/ hashtable!
        }
        
        Debug.Log("Stopped looking for path (left the while-loop)");
        
        // Plot your path to see if it makes sense. Note that path can only be seen in "Scene" window, not "Game" window
        for (int i = 0; i < path_of_points.Count-1; i++) // Debug.drawline draws a line between a start point and end point IN GLOBAL COORDS!
        { Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i+1] + Vector3.up, Color.magenta, 1000f); }
        //////////////////////////Catmull-Rom:
        SmoothSplineCatmullRom(path_of_points, 5);
        for (int j = 0; j < smooth_path_of_points.Count-1; j++)
        { Debug.DrawLine(smooth_path_of_points[j] + Vector3.up, smooth_path_of_points[j+1] + Vector3.up, Color.yellow, 1000f); }

        Debug.Log("Path of points contains :"+ path_of_points.Count + "points");
        Debug.Log("Smooth path contains :"+ smooth_path_of_points.Count + "points");

        //Spawn the LookAheadSphere
        //Instantiate(lookAheadSphere, transform.position, Quaternion.identity);
    }
    
    public void SmoothSplineCatmullRom(List<Vector3> originalPath, int subdivisionsPerSegment = 5)
    {
        if (originalPath.Count < 2)
            return; //originalPath;

        //List<Vector3> smoothed = new List<Vector3>();
        // Add first waypoint
        this.smooth_path_of_points.Add(originalPath[0]);

        for (int i = 0; i < originalPath.Count - 1; i++)
        {
            // p0: previous point (or same as p1 if none)
            Vector3 p0 = (i == 0) ? originalPath[i] : originalPath[i - 1];
            // p1: current
            Vector3 p1 = originalPath[i];
            // p2: next
            Vector3 p2 = originalPath[i + 1];
            // p3: next-next or same as p2 if none
            Vector3 p3 = (i + 2 < originalPath.Count) ? originalPath[i + 2] : originalPath[i + 1];
            
            // Generate Catmull-Rom points
            for (int step = 1; step <= subdivisionsPerSegment; step++)
            {
                float t = (float)step / (float)subdivisionsPerSegment;
                Vector3 newPoint = CatmullRom(p0, p1, p2, p3, t);
                this.smooth_path_of_points.Add(newPoint);
            }
        }
        return; //smooth_path_of_points;
    }

    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Standard Catmull-Rom formula
        // More references: https://www.iquilezles.org/www/articles/minispline/minispline.htm
        Vector3 a = 2f * p1;
        Vector3 b = p2 - p0;
        Vector3 c = 2f * p0 - 5f * p1 + 4f * p2 - p3;
        Vector3 d = -p0 + 3f * p1 - 3f * p2 + p3;
        return 0.5f * (a + (b * t) + (c * t * t) + (d * t * t * t));
    }
    
    private void FixedUpdate()
    {
        // WHILE LOOPS INSIDE Update()/ FixedUpdate() will keep FixedUpdate() from updating
        // effectively hanging the Unity Editor. FixedUpdate() already acts like a loop!
        
        // IMPORTANT NOTE ABOUT local vs global positions:
        // transform.position will give you the global position of the gameobject that this script is attached to
        // transform.localposition will give you the position of the gameobject in relation to its parent in the 
        // Hierarchy in the Unity Editor. In this case local pos of the car will be in relation to the parent, Map.
        
        /* /////////////////////////// RAYCASTING FOR RECOVERY FROM COLLISIONS
        var globalPosition = transform.position;

        var localPointTraveribility = obstacleMap?.GetLocalPointTraversibility(transform.localPosition);
        var globalPointTravesibility = obstacleMap?.GetGlobalPointTravesibility(transform.position);

        // How to calculate if something intersects the location of a box
        var overlapped = Physics.CheckBox(
            center: new Vector3(3f, 0f, 3f), // Global position to check
            halfExtents: new Vector3(1f, 0.1f, 1f) // Size of box (+- Vec in each direction)
        );

        // 'out's give shortest direction and distance to "uncollide" two objects.
        if (overlapped)
        {
            // Do your thing
        }
        // For more details https:docs.unity3d.com/ScriptReference/Physics.CheckBox.html
        // The Physics class has a bunch of static classes for all kinds of checks.

        // // This is how you access information about the terrain from a simulated laser range finder
        // // It might be wise to use this for error recovery, but do most of the planning before the race clock starts
        RaycastHit hit;
        float maxRange = 50f;
        if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
        {
            Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            // Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);
            // Debug.Log("Did Hit");
        }
        */ /////////////////////////// RAYCASTING FOR RECOVERY FROM COLLISIONS
        
        // Just some lines pointing to start and goal
        //Debug.DrawLine(globalPosition, mapManager.GetGlobalStartPosition(), Color.cyan); // Draw in global space
        //Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);

        if (smooth_path_of_points.Count < 2)
        {} //Most likely, the goal has been reached. So to not raise an index error, let's stop here
        else
        {
            //Execute your path here
            Vector3 goal_pos_global = mapManager.GetGlobalGoalPosition();
            Vector3 car_pos_global = transform.position;
            //Vector3 target_pos_global = lookAheadSphere.transform.position;
            Vector3 goal_error = goal_pos_global - car_pos_global;
            var distance_from_goal = goal_error.magnitude;
            var old_target_position = smooth_path_of_points[0];
            var target_position = smooth_path_of_points[1];
            Vector3 target_velocity = (target_position - old_target_position) / Time.fixedDeltaTime;
            Vector3 position_error = target_position - car_pos_global;
            Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;
            Vector3 vector_between_car_and_waypoint = car_pos_global - smooth_path_of_points[0];
            float current_speed = my_rigidbody.linearVelocity.magnitude;
            int[] perpendicular_angles = {85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95};
            float angle_to_waypoint =
                Vector3.SignedAngle(transform.forward, vector_between_car_and_waypoint,
                    Vector3.up); //transform.forward is the car's forward direction
            
            //Dynamic obstacle avoidance with RAYCASTS
            raycast_hit_positions.Clear();
            float scan_distance;
            
            //I'm working with a droplet-shape of rays. I want the front rays to be longer than the ones on the sides of the car.
            //Mapping function f(x) = 1-abs(x)/45. x is our angle i, f(x) is our scan distance.
        
            for (float i = -80f; i <= 80f; i+= 5f) { // 90/5=18 rays

                scan_distance = max_scan_distance - max_scan_distance*(Mathf.Abs(i) / 80f);
                Vector3 direction_i = Vector3.Normalize(Quaternion.Euler(0, i, 0)*transform.forward); //Quat.Euler(0,i,0)*transf.fwd rotates transf.fwd i degrees around y-axis
                Debug.DrawLine(transform.position, transform.position+(direction_i*scan_distance), Color.blue);
                
                RaycastHit hit_object; //declare hit-object
                if (Physics.Raycast(transform.position, direction_i, out hit_object, scan_distance))
                {   //obstacle detected close to vehicle, save the position where raycast hit it
                    raycast_hit_positions.Add(hit_object.point); //adding point of impact (in global coords) where ray hit the obstacle
                }
            }
        
            if (raycast_hit_positions.Count > 0) //We are currently looking at obstacles with our raycasts
            {
                waypoint_margin = 4f; //Increasing waypoint margin while maneuvering away from obstacle to not lose path
                foreach (Vector3 hit_pos in raycast_hit_positions)
                {   //Calculate steering input to obstacle, and then add its negative counterpart (scaled down) to steering inputs later
                
                    if (obstacle_avoiding_steering < obstacle_avoiding_steering_limit)
                    {
                        var old_waypoint = smooth_path_of_points[0];
                        Vector3 target_v = (hit_pos - old_waypoint) / Time.fixedDeltaTime;  //This is just the adapted PD
                        Vector3 pos_err = hit_pos - car_pos_global;                         //controller algorithm
                        Vector3 velocity_err = target_v - my_rigidbody.linearVelocity;      //used for steering away from obstacles
                        Vector3 desired_acc = k_p * pos_err + k_d * velocity_err;
                        obstacle_avoiding_steering += -Vector3.Dot(desired_acc, transform.right);
                        obstacle_avoiding_steering = Mathf.Clamp(obstacle_avoiding_steering,
                            -1*obstacle_avoiding_steering_limit, obstacle_avoiding_steering_limit);
                    }
                }
            }
            else //No obstacles nearby
            {
                obstacle_avoiding_steering = 0f;
                if (!car_is_perpendicular) // We increase waypoint_margin IF car_is_perpendicular = true OR we spot obstacles with raycast.
                { waypoint_margin = 3f; }  // We only decrease waypoint_margin IF car_is_perpendicular = false AND we do not spot obstacles w. raycast.
            }
            //Dynamic obstacle avoidance above
            Debug.Log("Avoidance steering: "+obstacle_avoiding_steering + "-------------------");
            
            if (distance_from_goal < waypoint_margin) //We have reached the goal
            {
                Debug.Log("Goal reached! :D");
                smooth_path_of_points.Clear(); //empty the list
            }
            else if (Vector3.Magnitude(car_pos_global - smooth_path_of_points[0]) < waypoint_margin) 
            {   // we have reached the current target position
                //Debug.Log("Reached next target position!");
                smooth_path_of_points.RemoveAt(0); //remove index 0 so we can go to next target
            }
            else // We are driving to the next waypoint
            {
                if (car_is_perpendicular)
                {
                    //Drive out of this situation
                    steering = Vector3.Dot(desired_acceleration, transform.right);
                    accel = (my_rigidbody.linearVelocity.magnitude > speed_limit) ? -1f : 10f;
                    //?: "ternary operator". Assigns accel the value to left of : if condition in front of ? is true, otherwise assigns accel = value to right of :

                    Debug.DrawLine(transform.position, target_position + Vector3.up, Color.red);
                    //Debug.Log("Steering: "+steering+" Velocity: "+current_speed+" Acceleration: "+accel);

                    m_Car.Move(steering, accel, accel, 0f);

                    //Check if still perpendicular, otherwise this.car_is_perpendicular = false;
                    if (!perpendicular_angles.Contains((int)Mathf.Round(Mathf.Abs(angle_to_waypoint))))
                    {
                        car_is_perpendicular = false;
                        Debug.Log("No longer perpendicular to path");
                        Debug.Log("angle_to_waypoint = "+angle_to_waypoint);
                        if (raycast_hit_positions.Count == 0) // We increase waypoint_margin IF car_is_perpendicular = true OR we spot obstacles with raycast.
                        { this.waypoint_margin = 3f; } // We only decrease waypoint_margin IF car_is_perpendicular = false AND we do not spot obstacles w. raycast.
                    }
                }
                else //Driving between waypoints along the path, car not perpendicular to path, path hasn't ended yet
                {
                    steering = Vector3.Dot(desired_acceleration, transform.right);
                    accel = (my_rigidbody.linearVelocity.magnitude > speed_limit) ? -1f : Vector3.Dot(desired_acceleration, transform.forward);
                    //?: "ternary operator". Assigns accel the value to left of : if condition in front of ? is true, otherwise assigns accel = value to right of :

                    Debug.DrawLine(transform.position, target_position + Vector3.up, Color.red);
                    //Debug.Log("Steering: "+steering+" Velocity: "+current_speed+" Acceleration: "+accel);
                    // Control the car via mCar.Move()
                    // Found via testing:
                    // accel: only takes positive values, accelerates car (negative values do nothing)
                    // footbrake: we use accel as footbrake value in Move(): when accel<0, footbrake will slow down/ reverse the car
                    // handbrake does what you'd imagine
                    // steering: positive values steer the car to the right, negative values steer to the left
                    m_Car.Move(steering + obstacle_avoiding_steering, accel, accel, 0f);

                    //When the car is perpendicular to the waypoint, it doesn't know whether to accelerate forwards or backwards so it gets stuck
                    if (!car_is_perpendicular) //We have to check for this
                    {
                        //Check if perpendicular, if it is then this.car_is_perpendicular = true;
                        if (perpendicular_angles.Contains((int)Mathf.Round(Mathf.Abs(angle_to_waypoint))))
                        {
                            car_is_perpendicular = true;
                            this.waypoint_margin = 6f;
                            Debug.Log("Car is perpendicular to path");
                            Debug.Log("angle_to_waypoint = "+angle_to_waypoint);
                        }
                    }
                }
            }
        }
    }
}
