using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Numerics;
using FormationGame;
using Newtonsoft.Json;
using Scripts.Map;
using Scripts.Game;
using UnityEditor;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Vehicle;

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : MonoBehaviour
{
    private DroneController m_Drone;
    //Dict of all waypoints used by all drones, and the orientations at those waypoints:
    public static Dictionary<Vector3, HashSet<float>> globalPathRegistry = new Dictionary<Vector3, HashSet<float>>();
    public static int droneCounter = 0; //This field belongs to the class/type, not to any specific object of the class

    //It is used to give an index to the specific drone clone that has this script attached.
    public int myDroneIndex; //This drone's specific index in array of agents m_OtherDrones (includes this drone!)
    private float waiting_multiplier = 0f;
    private bool start_moving = false;
    public bool goal_reached = false;

    private Rigidbody my_rigidbody;
    private MapManager m_MapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherDrones;
    private List<MultiVehicleGoal> m_CurrentGoals;

    private ImprovePath m_improvePath;
    private IntersectionDrone m_Intersection;

    public List<StateNode> path_of_nodes = new List<StateNode>();

    public List<Vector3> path_of_points = new List<Vector3>();

    public int currentPathIndex = 1;
    public Vector3 target_position;
    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public float k_p = 1f;
    public float k_d = 2f;
    private bool isStuck = false;
    private int timeStuck = 0;

    //For driving:
    private float goalpoint_margin = 3f; //Serves as a means of checking if we're close enough to goal while planning path
    public float distToPoint = 2.5f; // min distance to go to the next point in path
    public bool hasToStop; // for intersection
    private bool droneCloseInFront = false; //for when another drone is close in front to this drone, then we should stop and allow the drone in front to move away
    private int timeStoppedForFrontDrone = 0;
    private bool disableFrontCheck = false;
    private int timeFrontCheckDisabled = 0;
    Vector3 desired_acceleration;

    //private bool obstacles_close = false;
    private List<Vector3> raycast_disk_hit_positions = new List<Vector3>();
  

    public List<GameObject> targetObjects;
    public List<GameObject> teamVehicles;

    //Raycast
    RaycastHit hitRight; RaycastHit hitLeft; RaycastHit hitStraight;
    RaycastHit hitBackRight; RaycastHit hitBackLeft; RaycastHit hitBack;
    float maxRangeClose = 7f;
    bool obsRightClose, obsLeftClose, obsStraightClose, obsBackRightClose, obsBackLeftClose, obsBackClose;

    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        my_rigidbody = GetComponent<Rigidbody>();
        m_improvePath = new ImprovePath();
        m_Intersection = new IntersectionDrone();

        m_MapManager = FindFirstObjectByType<MapManager>();
        float cell_size = 2.8f;
        Vector3 cell_scale = Vector3.one * cell_size;
        //cell_scale.y = 0f; //cell_scale is now [3,0,3]
        m_ObstacleMap = ObstacleMap.Initialize(m_MapManager, new List<GameObject>(), cell_scale);
        m_ObstacleMap.margin = Vector3.one * 2f; // (Changing cell margins, do they work?)
        StateNode.cell_size = cell_size;

        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        m_CurrentGoals = gameManagerA2.GetGoals(this.gameObject); // This drone's goal.
        teamVehicles = gameManagerA2.GetGroupVehicles(this.gameObject); //Other vehicles in a Group with this vehicle
        m_OtherDrones = GameObject.FindGameObjectsWithTag("Player"); //All vehicles

        // Note that this array will have "holes" when objects are destroyed. For initial planning they should work.
        // If you don't like the "holes", you can re-fetch this during fixed update.
        // Equivalent ways to find all the targets in the scene
        targetObjects = m_CurrentGoals.Select(goal => goal.GetTargetObject()).ToList(); //targetObjects is a list of one element for some reason
        // You can also fetch other types of objects using tags, assuming the objects you are looking for HAVE tags :).
        // Feel free to refer to any examples from previous assignments.

        ////////////////////////// Plan your path here
        myDroneIndex = droneCounter % m_MapManager.startPositions.Count; //Index of this specific drone
        droneCounter++; //myDroneIndex is used to find the specific start_pos and goal_pos of this drone

        Vector3 start_pos_global = m_MapManager.startPositions[this.myDroneIndex];
        Vector3 goal_pos_global = m_MapManager.targetPositions[this.myDroneIndex];

        start_pos_global = SnapToGridCenter(start_pos_global, cell_size); // Update to center of cell

        /*
        // Calculate the starting angle of the drone
        // Get the current Y rotation angle (in degrees)
        float currentAngle = m_Drone.transform.rotation.eulerAngles.y;

        // Find the closest multiple of 30 degrees
        float closestMultipleOf30 = Mathf.Round(currentAngle / 30f) * 30f;

        // Ensure the angle is within 0 to 360 degrees
        float resultAngle = closestMultipleOf30 % 360f;

        // If the result is negative, ensure it's within the positive range (0 to 360)
        if (resultAngle < 0)
        {
            resultAngle += 360f;
        }
        */


        PriorityQueue Q = new PriorityQueue();
        Dictionary<Vector3Int, StateNode> visited_nodes = new Dictionary<Vector3Int, StateNode>();
        //StateNode start_node = new StateNode(start_pos_global, resultAngle, start_pos_global, goal_pos_global, null, m_MapManager, m_ObstacleMap, myDroneIndex);
        StateNode start_node = new StateNode(start_pos_global, 0f, start_pos_global, goal_pos_global, null, m_MapManager, m_ObstacleMap, myDroneIndex);
        Q.Enqueue(start_node);

        while (Q.Count != 0)
        {
            var current_node = Q.Dequeue();
            if (!visited_nodes.ContainsKey(current_node.cell_position))
            {
                visited_nodes.Add(current_node.cell_position, current_node);
            }

            if (Vector3.Distance(current_node.world_position, goal_pos_global) <= this.goalpoint_margin) //We have reached the goal, time to create the path
            {
                current_node.fillPaths(this.path_of_nodes, this.path_of_points);
                //Now the path_of_nodes and path_of_points are ready to be executed. We can either use StateNodes or Vector3s in the controller.
                break; // BREAK OUT OF WHILE LOOP, WE'RE DONE HERE
            }

            //else we keep looking:
            List<StateNode> new_nodes = current_node.makeChildNodes(visited_nodes, Q, m_MapManager, m_ObstacleMap, cell_scale.z, "drone");
            foreach (StateNode n in new_nodes)
            {
                Q.Enqueue(n);
            } //Add all new nodes to the queue
        }


        //If path.Count() != 0, i.e. we found a path for this drone, add all its waypoints to the globalPathRegistry
        if (path_of_nodes.Count() != 0)
        {
            foreach (StateNode node in path_of_nodes)
            {
                if (!globalPathRegistry.ContainsKey(node.world_position)) //If this world_pos is not in globalPathRegistry already
                {
                    globalPathRegistry.Add(node.world_position, new HashSet<float>()); //Add new world_pos - Hashset<float> pair to globalPathRegistry
                }
                globalPathRegistry[node.world_position].Add(node.orientation); //Add this node's orientation to Hashset corresponding to same node's world_pos
                /* The key-value pairs in globalPathRegistry are Vector3 and Hashset<float>. The reason for this is so we can keep track of positions used
                in paths of other drones while planning the path for this drone. We keep a Hashset of all orientations of every node at the corresponding
                world_pos in order to be able to check if a new node is at a world_pos that's already been used, AND IF SO, if it previously was used
                with an orientation opposite to the new node's orientation. If that is the case, we discard the new node to not allow overlapping paths
                in opposite directions. This is to avoid head on collisions that cars may have trouble getting out of and drones will suffer a lot from.*/
            }

            // Plot your path to see if it makes sense. Note that path can only be seen in "Scene" window, not "Game" window
            for (int i = 0; i < path_of_points.Count - 1; i++)
            {
                Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f);
            }
            
            //Initialize target positions
            target_position = path_of_points[1];
            old_target_pos = transform.position;
        }

        StartCoroutine(wait()); //Wait to begin driving
    }

    private IEnumerator wait()
    {
        yield return new WaitForSeconds(myDroneIndex * waiting_multiplier);

        //Fixedupdate runs while start is waiting so we need to define a flag that forbids update from driving while Start is waiting
        this.start_moving = true;
    }

    private void FixedUpdate()
    {
        if (goal_reached == true) StopTheDrone(); //Make the drone stop after reaching goal.

        else if (start_moving == true && path_of_points.Count != 0 && currentPathIndex < path_of_points.Count)
        {
            //Checks if we have any driving left to do
            Vector3 new_velocity = Vector3.zero;

            DriveAndRecover(new_velocity); //follow path, recover if stuck
        }
    }

    private void OnDrawGizmos() // This method is called by the Unity Editor everytime FixedUpdate() runs.
    {
        // It must not be called by us during runtime, that will raise an error
        Debug.DrawLine(transform.position + Vector3.up * 2f, target_position + Vector3.up * 2f, Color.red);  // Shows where we're aiming to follow

        if (droneCloseInFront)
        {
            Handles.color = Color.red; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a drone that is stopping for intersection
        }
        else if (hasToStop)
        {
            Handles.color = Color.blue; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a drone that is stopping for intersection
        }
        else if (goal_reached)
        {
            Handles.color = Color.green; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a drone that is stopping for intersection

        }

    }

    //Follow waypoints and do collision recovery
    private void DriveAndRecover(Vector3 orca_velocity)
    {
        //Detect obstacles and close drones in front
        UpdateRaycast();

        if (droneCloseInFront == true) //Make the drone stop for drone in front to have a chance to drive away without collision
        { //Keep driving only if the raycast in front returns false.
            StopTheDrone();
            //timeStoppedForFrontDrone += 1;
            //if (timeStoppedForFrontDrone >= 500) disableFrontCheck = true; //If we've been waiting for more than 10 seconds, disable front checks so we can do collision recovery
            return;
        }

        // Update target and old target to the current index in the path
        if (target_position != path_of_points[currentPathIndex])
        {
            target_position = path_of_points[currentPathIndex];
            old_target_pos = path_of_points[currentPathIndex - 1];
            target_velocity = (target_position - old_target_pos) / 2f;
        }

        hasToStop = m_Intersection.HasToStop(m_Drone, m_OtherDrones); // Check if if we have to stop

        //if (!isStuck) // If the drone is not stuck //isStuck is not relevant for drones?
        {
            if (hasToStop) // If I have to stop for intersection, brake
            {
                StopTheDrone();
            }

            else
            {             
                PdTracker(); // Update acceleration and steering values for driving

                m_Drone.Move(desired_acceleration.x, desired_acceleration.z);
            }

            // Update currentPathIndex
            if (Vector3.Distance(path_of_points[currentPathIndex], transform.position) < distToPoint)
            {
                currentPathIndex++;

                if (currentPathIndex == path_of_points.Count - 1) { distToPoint = 0.25f; } //Changing distToPoint to be smaller when next waypoint is the goal

                if (currentPathIndex == path_of_points.Count) goal_reached = true;
            }

            if (raycast_disk_hit_positions.Count() > 0) //We have another agent in our raycast disk
            {   //Increasing distToPoint so we can validate waypoints from further away while avoiding obstacles/ other agents
                distToPoint = 4;

                foreach (Vector3 hit_pos in raycast_disk_hit_positions)
                {
                    //Calculate steering input to obstacle, and then add its negative counterpart (scaled down) to steering inputs later

                    if (my_rigidbody.linearVelocity.magnitude < 4f) //Don't accelerate away so fast that the speed sends us into a wall somewhere else
                    {
                        //This^ if statemenet could maybe depend on how many hit_pos we are trying to accelerate away from
                        //Only take first 4 hit_pos into account to avoid having too many calls to Move() sending the drone away into a wall? :)
                        target_position = hit_pos;
                        PdTracker();
                        m_Drone.Move(-desired_acceleration.x, -desired_acceleration.z);
                    }
                }
                //return;
            }
            else distToPoint = 2.5f; //resetting distToPoint to normal value since we are not avoiding any obstacles atm

            /* Not relevant for drones?
            // If you're barely moving it means you may be stuckstuck
            if (my_rigidbody.linearVelocity.magnitude < 0.5f)
            {
                timeStuck += 1;
                if (timeStuck > 100) // If you're not moving for too long you're stuck
                {
                    isStuck = true;
                }
            }
            else timeStuck = 0;
            */
        }

        /* Not relevant for drones?
        else //if stuck:
        {
            if (!hasToStop) // Check if you're stuck or if your're waiting for another drone to go
            {
                // If you have an obstacle behind you go forward
                //if (obsBackClose || obsBackRightClose || obsBackLeftClose) m_Drone.Move(-desired_acceleration.x, -desired_acceleration.z);
                //else m_Drone.Move(-desired_acceleration.x, -desired_acceleration.z); //go backwards
                m_Drone.Move(-my_rigidbody.linearVelocity.x, -my_rigidbody.linearVelocity.z);   
                timeStuck -= 1;
                if (timeStuck == 0) isStuck = false;
            }
            else // you're not stuck you're just waiting at an intersection
            {
                timeStuck = 0;
                isStuck = false;
                StopTheDrone();
            }
        }
        */
    }

    /*
     * Pd_tracker to update steering and acceleration based on target_position and old_target_position
     */
    private void PdTracker()
    {
        float distance = Vector3.Distance(target_position, transform.position);

        // Scale k_p and k_d based on distance between 1 and 10
        float scaleFactor = Mathf.Clamp(distance / 2f, 1f, 1f);  // Adjust 2f(first one) to control sensitivity, bigger means accelarate more abruptly
        float k_p_dynamic = Mathf.Lerp(3f, 10f, scaleFactor / 1f);
        float k_d_dynamic = Mathf.Lerp(4f, 8f, scaleFactor / 1f);

        float k_v = Mathf.Lerp(1f, 2f, scaleFactor / 8f);  // New gain factor for velocity feedback
        Vector3 velocity_damping = -k_v * my_rigidbody.linearVelocity;

        // a PD-controller to get desired acceleration from errors in position and velocity
        Vector3 position_error = target_position - transform.position;
        Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
        desired_acceleration = k_p_dynamic * position_error + k_d_dynamic * velocity_error + velocity_damping;

        Debug.DrawLine(transform.position, transform.position + desired_acceleration.normalized * 5, Color.yellow);
    }

    /*
     * Detect obstacles around the drone
     * Used to add steering if needed
     * Used to move forward or backwars if stuck
     */
    private void UpdateRaycast()
    {
        // Rotate the drone's forward vector by �30� to get left/right directions for the raycast

        Vector3 directionRight = Quaternion.Euler(0, -30, 0) * transform.forward;
        Vector3 directionLeft = Quaternion.Euler(0, 30, 0) * transform.forward;
        Vector3 directionBackLeft = Quaternion.Euler(0, 150, 0) * transform.forward;
        Vector3 directionBackRight = Quaternion.Euler(0, -150, 0) * transform.forward;
        Vector3 directionBack = Quaternion.Euler(0, 180, 0) * transform.forward;

        obsRightClose = Physics.Raycast(transform.position, directionRight, out hitRight, maxRangeClose);
        obsLeftClose = Physics.Raycast(transform.position, directionLeft, out hitLeft, maxRangeClose);
        obsStraightClose = Physics.Raycast(transform.position, transform.TransformDirection(new Vector3(0, 0, 1)),
            out hitStraight, maxRangeClose);
        obsBackRightClose = Physics.Raycast(transform.position, directionBackRight, out hitBackRight, maxRangeClose);
        obsBackLeftClose = Physics.Raycast(transform.position, directionBackLeft, out hitBackLeft, maxRangeClose);
        obsBackClose = Physics.Raycast(transform.position, directionBack, out hitBack, maxRangeClose);

        //Check for AGENTS close in front
        if (!disableFrontCheck)
        {
            droneCloseInFront = false;
            float front_scan_distance = 10f;
            Vector3 current_waypoint = path_of_points[currentPathIndex]; //+Vector3.up*1.8f is added to not have the ray point into the ground
            Vector3 from_agent_to_waypoint = current_waypoint+Vector3.up*1.8f - transform.position; //The actual waypoints are below and in front of the drone
            //droneCloseInFront is set to true if any of the two rays we cast return true.

            //The rays are: first ray is in dir of linearvelocity, 2nd ray is in dir of waypoint,
            //3rd, 4th are slightly rotated from dir of waypoint

            RaycastHit velocity_hit_obj;
            Debug.DrawRay(transform.position+Vector3.up, my_rigidbody.linearVelocity.normalized * front_scan_distance, Color.black);
            if(Physics.Raycast(transform.position + Vector3.up, my_rigidbody.linearVelocity.normalized,
                out velocity_hit_obj, front_scan_distance, LayerMask.GetMask("Default")))
            {
                if (!velocity_hit_obj.rigidbody.GetComponent<AIP2TrafficDrone>().goal_reached) //If the agent we hit has not reached its goal yet
                {
                    droneCloseInFront = true;
                }
            }

            for (float j = -6f; j <= 6f; j += 3f)
            {
                RaycastHit waypoint_hit_obj;
                Debug.DrawRay(transform.position+Vector3.up, Quaternion.Euler(0f, j, 0f) * from_agent_to_waypoint.normalized * front_scan_distance, Color.black);
                if(Physics.Raycast(transform.position + Vector3.up, 
                       Quaternion.Euler(0f, j, 0f)*from_agent_to_waypoint.normalized, out waypoint_hit_obj, 
                       front_scan_distance, LayerMask.GetMask("Default")))
                {

                    if (!waypoint_hit_obj.rigidbody.GetComponent<AIP2TrafficDrone>().goal_reached) //If the agent we hit has not reached its goal yet
                    {
                        droneCloseInFront = true;
                        break;
                    }
                }
            }
        }
        /*else
        {
            droneCloseInFront = false;
            timeStoppedForFrontDrone = 0;

            timeFrontCheckDisabled += 1;
            if (timeFrontCheckDisabled > 100) //If front check has been disabled for longer than 2 seconds, re-enable it
            {
                disableFrontCheck = false;
                timeFrontCheckDisabled = 0;
            }
        }*/
        
        //Dynamic obstacle avoidance with RAYCASTS
        raycast_disk_hit_positions.Clear();
        float disk_scan_distance = 2.5f;

        for (float i = -180f; i <= 180f; i += 10f) //Rays go from -180 degrees to +180 degrees. One ray per 20 degrees - 18 rays.
        {
            Vector3 direction_i = Vector3.Normalize(Quaternion.Euler(0, i, 0) * transform.forward); //Quat.Euler(0,i,0)*transf.fwd rotates transf.fwd i degrees around y-axis
            Debug.DrawLine(transform.position, transform.position + (direction_i * disk_scan_distance), Color.white);

            RaycastHit hit_object; //declare hit-object
            if (Physics.Raycast(transform.position, direction_i, out hit_object, disk_scan_distance)) //, LayerMask.GetMask("Default")))
            {
                //Agent detected close to vehicle, save the position where raycast hit it
                raycast_disk_hit_positions.Add(hit_object.point); //adding point of impact (in global coords) where ray hit the obstacle
            }
        }
    }

    // Function to round starting coordinates to the center of the cell
    private Vector3 SnapToGridCenter(Vector3 position, float cell_size)
    {
        return new Vector3(
            Mathf.Round(position.x / cell_size) * cell_size + cell_size / 2f,
            position.y, // Keep the original Y value
            Mathf.Round(position.z / cell_size) * cell_size + cell_size / 2f
        );
    }

    private void StopTheDrone()
    {   //Method that stops the drone without using the handbrake in order to avoid the handbrake bug
        //The handbrake bug causes the drone to have trouble resetting the handbrake to 0 after using the handbrake
        //This makes the drone unable to start accelerating again even after setting handbrake=0f, accel>0.
        m_Drone.Move(-my_rigidbody.linearVelocity.x, -my_rigidbody.linearVelocity.z);

    }

}

