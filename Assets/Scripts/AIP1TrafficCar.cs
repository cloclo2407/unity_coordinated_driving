using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Numerics;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Newtonsoft.Json;
using Scripts.Map;
using Scripts.Game;
using UnityEditor;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using static alglib;
using static alglib.minqp;
using Vector2 = UnityEngine.Vector2;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : MonoBehaviour
{
    //Dict of all waypoints used by all cars, and the orientations at those waypoints:
    public static Dictionary<Vector3, HashSet<StateNode>> globalPathRegistry = new Dictionary<Vector3, HashSet<StateNode>>();
    public static int carCounter = 0; //This field belongs to the class/type, not to any specific object of the class

    //It is used to give an index to the specific car clone that has this script attached.
    public int myCarIndex; //This car's specific index in array of agents m_OtherCars (includes this car!)
    private float waiting_multiplier = 0f;
    private bool start_moving = false;
    public bool goal_reached = false;

    private float cell_size = 3f;
    private Vector3 cell_scale = Vector3.one * 3f;

    private CarController m_Car; // the car controller we want to use
    private BoxCollider carCollider; //unused BoxCollider
    private Rigidbody my_rigidbody;
    private MapManager m_MapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private List<MultiVehicleGoal> m_CurrentGoals;

    private Intersection m_Intersection;
    
    public List<StateNode> path_of_nodes = new List<StateNode>();
    public List<Vector3> path_of_points = new List<Vector3>();
    public List<List<Vector3>> list_paths = new List<List<Vector3>>();

    public int currentPathIndex = 1;
    private int currentPath = 0;
    public Vector3 target_position;
    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public float k_p = 1f;
    public float k_d = 2f;
    private bool isStuck = false;
    private int timeStuck = 0;

    //For driving:
    private float goalpoint_margin = 3.8f; //Serves as a means of checking if we're close enough to goal while planning path
    public float distToPoint = 4f; // min distance to go to the next point in path
    public bool hasToStop; // for intersection
    private bool carCloseInFront; //for when another car is close in front to this car, then we should stop and allow the car in front to move away
    private int timeStoppedForFrontCar = 0;
    private bool disableFrontCheck = false;
    private int timeFrontCheckDisabled = 0;
    
    //private bool obstacles_close = false;
    private List<Vector3> raycast_hit_positions = new List<Vector3>();

    private float obstacle_avoiding_steering = 3f; //Addition to steering input calculated via raycasts of environment

    public float steering;
    public float acceleration;
    public List<GameObject> targetObjects;
    public List<GameObject> teamVehicles;

    //Raycast
    RaycastHit hitRight; RaycastHit hitLeft; RaycastHit hitStraight;
    RaycastHit hitBackRight; RaycastHit hitBackLeft; RaycastHit hitBack;
    float maxRangeClose = 7f;
    bool obsRightClose, obsLeftClose, obsStraightClose, obsBackRightClose, obsBackLeftClose, obsBackClose;

    private void Start()
    {
        m_Car = GetComponent<CarController>();
        my_rigidbody = GetComponent<Rigidbody>();

        m_Intersection = new Intersection();

        m_MapManager = FindFirstObjectByType<MapManager>();
        m_ObstacleMap = ObstacleMap.Initialize(m_MapManager, new List<GameObject>(), cell_scale);
        m_ObstacleMap.margin = Vector3.one * 2f; // (Changing cell margins, do they work?)
        StateNode.cell_size = cell_size;

        m_OtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles

        myCarIndex = carCounter % m_MapManager.startPositions.Count; //Index of this specific car
        carCounter++; //myCarIndex is used to find the specific start_pos and goal_pos of this car

        ComputeAllPaths();
        path_of_points = list_paths[0];

        // Plot your path to see if it makes sense. Note that path can only be seen in "Scene" window, not "Game" window
        for (int i = 0; i < path_of_points.Count - 1; i++) {
            // Debug.drawline draws a line between a start point and end point IN GLOBAL COORDS!
            Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f);
        }   
      
        target_position = path_of_points[1];
        old_target_pos = transform.position;
        
        StartCoroutine(wait()); //Wait to begin driving
    }
    
    private IEnumerator wait()
    { 
        yield return new WaitForSeconds(myCarIndex * waiting_multiplier);
        //Fixedupdate runs while start is waiting so we need to define a flag that forbids update from driving while Start is waiting
        this.start_moving = true;
    }

    private void FixedUpdate()
    {
        if (goal_reached == true) StopTheCar(); //Make the car stop after reaching goal.
        
        else if (start_moving == true && path_of_points.Count != 0 && currentPathIndex < path_of_points.Count)
        {
            DriveAndRecover(); //follow path, recover if stuck
        }
    }

    private void OnDrawGizmos() // This method is called by the Unity Editor everytime FixedUpdate() runs.
    {
        // It must not be called by us during runtime, that will raise an error

        Debug.DrawLine(transform.position + Vector3.up * 2f, target_position + Vector3.up * 2f, Color.red);  // Shows where we're aiming to follow

        if (carCloseInFront)
        {
            Handles.color = Color.red; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a car that is stopping for intersection
        }
        else if (hasToStop)
        {
            Handles.color = Color.blue; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a car that is stopping for intersection
        }
        else if (goal_reached)
        {
            Handles.color = Color.green; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a car that is stopping for intersection
        }
    }

    //Follow waypoints and do collision recovery
    private void DriveAndRecover() 
    { 
        //Detect obstacles and close cars in front
        UpdateRaycast();

        if (carCloseInFront == true) //Make the car stop for car in front to have a chance to drive away without collision
        { //Keep driving only if the raycast in front returns false.
            StopTheCar(); 
            timeStoppedForFrontCar += 1;
            if (timeStoppedForFrontCar >= 500) disableFrontCheck = true; //If we've been waiting for more than 10 seconds, disable front checks so we can do collision recovery
            return;
        }
        
        // Update target and old target to the current index in the path
        if (target_position != path_of_points[currentPathIndex]) 
        {
            target_position = path_of_points[currentPathIndex];
            old_target_pos = path_of_points[currentPathIndex - 1];
            target_velocity = (target_position - old_target_pos) / 2f;
        }

        hasToStop = m_Intersection.HasToStop(m_Car, m_OtherCars); // Check if if we have to stop

        if (!isStuck) // If the car is not stuck
        {
            if (hasToStop) // If I have to stop for intersection, brake
            {
                StopTheCar();
            }

            else
            {
                PdTracker(); // Update acceleration and steering values for driving

                // Turn if you're too close to an obstacle
                if (acceleration > 0)
                {
                    if (obsLeftClose) steering -= obstacle_avoiding_steering;
                    else if (obsRightClose) steering += obstacle_avoiding_steering;
                }
                else
                {
                    if (obsBackLeftClose) steering += obstacle_avoiding_steering;
                    else if (obsBackRightClose) steering -= obstacle_avoiding_steering;
                }  
                m_Car.Move(steering, acceleration, acceleration, 0f);
            }

            // Update currentPathIndex
            if (Vector3.Distance(path_of_points[currentPathIndex], transform.position) < distToPoint)
            {
                currentPathIndex++;          

                if (currentPathIndex == path_of_points.Count - 1) {distToPoint = 2; } //Changing distToPoint to be smaller when next waypoint is the goal

                // Can set a bigger distance to help the beginning
                else if (currentPathIndex < 4)
                {
                    distToPoint = 4f;
                }
                else { distToPoint = 4f; } //reset

                if (currentPathIndex == path_of_points.Count)
                {
                    if (currentPath < list_paths.Count - 1)
                    {
                        currentPath++;
                        path_of_points = list_paths[currentPath];
                        currentPathIndex = 1;
                    }
                    else goal_reached = true;
                }
            }

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
        }

        else //if stuck:
        {
            if (!hasToStop) // Check if you're stuck or if your're waiting for another car to go
            {
                // If you have an obstacle behind you go forward
                if (obsBackClose || obsBackRightClose || obsBackLeftClose) m_Car.Move(0f, 10f, 10f, 0f);
                else m_Car.Move(0f, -10f, -10f, 0f); //go backwards

                timeStuck -= 1;
                if (timeStuck == 0) isStuck = false;
            }
            else // you're not stuck you're just waiting at an intersection
            {
                timeStuck = 0;
                isStuck = false;
                StopTheCar();
            }
        }
    }
    
    /*
     * Pd_tracker to update steering and acceleration based on target_position and old_target_position
     */
    private void PdTracker()
    {
        float distance = Vector3.Distance(target_position, transform.position);

        // Scale k_p and k_d based on distance between 1 and 10
        float scaleFactor = Mathf.Clamp(distance / 2f, 1f, 1f);  // Adjust 2f(first one) to control sensitivity, bigger means accelarate more abruptly
        float k_p_dynamic = Mathf.Lerp(3f, 4f, scaleFactor / 1f);
        float k_d_dynamic = Mathf.Lerp(4f, 4f, scaleFactor / 1f);

        float k_v = Mathf.Lerp(1f, 2f, scaleFactor / 8f);  // New gain factor for velocity feedback
        Vector3 velocity_damping = -k_v * my_rigidbody.linearVelocity;

        // a PD-controller to get desired acceleration from errors in position and velocity
        Vector3 position_error = target_position - transform.position;
        Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
        Vector3 desired_acceleration = k_p_dynamic * position_error + k_d_dynamic * velocity_error + velocity_damping;

        steering = Vector3.Dot(desired_acceleration, transform.right);
        acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        Debug.DrawLine(transform.position, transform.position + desired_acceleration.normalized * 5, Color.yellow);
    }

    /*
     * Detect obstacles around the car
     * Used to add steering if needed
     * Used to move forward or backwars if stuck
     */
    private void UpdateRaycast()
    {
        // Rotate the car's forward vector by �30� to get left/right directions for the raycast
        Vector3 directionRight = Quaternion.Euler(0, -30, 0) * transform.forward;
        Vector3 directionLeft = Quaternion.Euler(0, 30, 0) * transform.forward;
        Vector3 directionBackLeft = Quaternion.Euler(0, 150, 0) * transform.forward;
        Vector3 directionBackRight = Quaternion.Euler(0, -150, 0) * transform.forward;
        Vector3 directionBack = Quaternion.Euler(0, 180, 0) * transform.forward;

        obsRightClose = Physics.Raycast(transform.position, directionRight, out hitRight, maxRangeClose);
        obsLeftClose = Physics.Raycast(transform.position, directionLeft, out hitLeft, maxRangeClose);
        obsStraightClose = Physics.Raycast(transform.position, transform.TransformDirection(new Vector3(0, 0, 1)), out hitStraight, maxRangeClose);
        obsBackRightClose = Physics.Raycast(transform.position, directionBackRight, out hitBackRight, maxRangeClose);
        obsBackLeftClose = Physics.Raycast(transform.position, directionBackLeft, out hitBackLeft, maxRangeClose);
        obsBackClose = Physics.Raycast(transform.position, directionBack, out hitBack, maxRangeClose);

        //Check for AGENTS close in front
        if (!disableFrontCheck)
        {
            carCloseInFront = Physics.Raycast(transform.position+Vector3.up, transform.forward, 10f, LayerMask.GetMask("Default"));
            Debug.DrawRay(transform.position+Vector3.up, transform.forward*10f, Color.black);
        }
        else
        {
            carCloseInFront = false;
            timeStoppedForFrontCar = 0;
            
            timeFrontCheckDisabled += 1;
            if (timeFrontCheckDisabled > 100) //If front check has been disabled for longer than 2 seconds, re-enable it
            {
                disableFrontCheck = false;
                timeFrontCheckDisabled = 0;
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

    private void StopTheCar()
    {   //Method that stops the car without using the handbrake in order to avoid the handbrake bug
        //The handbrake bug causes the car to have trouble resetting the handbrake to 0 after using the handbrake
        //This makes the car unable to start accelerating again even after setting handbrake=0f, accel>0.
                
        if (my_rigidbody.linearVelocity.magnitude > 0.005f)
        {
            if (Vector3.Dot(my_rigidbody.transform.forward, my_rigidbody.linearVelocity) < 0)
            { //If the dot product is negative, the vectors point in opposite directions and that means
              //linearVelocity is pointing backwards, therefore we should accelerate forwards
                m_Car.Move(0f, 1f, 0f, 0f);
            }
            else
            { //If the dot product is positive, the vectors point in the same direction and that means
              //linearVelocity is pointing forwards, therefore we should accelerate backwards
                m_Car.Move(0f, 0f, -1f, 0f);
            }
        }
        else
        { //If the velocity magnitude is less than 0.005f, we consider the car stopped
            m_Car.Move(0f,0f,0f,0f);
        }
    }

    /*
     * Execute A* algo to compute a path from start_pos_global to goal_pos_global starting in orientation resultAngle
     */
    private List<Vector3> ComputePath(Vector3 start_pos_global, Vector3 goal_pos_global, float startAngle, out float finishAngle)
    {
        finishAngle = -10f;
        List<StateNode> path_nodes = new List<StateNode>();
        List<Vector3> path_points = new List<Vector3>();
        PriorityQueue Q = new PriorityQueue();
        Dictionary<Vector3Int, StateNode> visited_nodes = new Dictionary<Vector3Int, StateNode>();
        StateNode start_node = new StateNode(start_pos_global, startAngle, start_pos_global, goal_pos_global, null, m_MapManager, m_ObstacleMap, myCarIndex);
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
                current_node.fillPaths(path_nodes, path_points);
                //Now the path_of_nodes and path_of_points are ready to be executed. We can either use StateNodes or Vector3s in the controller.
                break; // BREAK OUT OF WHILE LOOP, WE'RE DONE HERE
            }

            //else we keep looking:
            List<StateNode> new_nodes = current_node.makeChildNodes(visited_nodes, Q, m_MapManager, m_ObstacleMap, cell_scale.z, "car");
            foreach (StateNode n in new_nodes)
            {
                Q.Enqueue(n);
            } //Add all new nodes to the queue
        }


        //If path.Count() != 0, i.e. we found a path for this car, add all its waypoints to the globalPathRegistry
        if (path_nodes.Count() != 0)
        {
            foreach (StateNode node in path_nodes)
            {
                if (!globalPathRegistry.ContainsKey(node.world_position)) //If this world_pos is not in globalPathRegistry already
                {
                    globalPathRegistry.Add(node.world_position, new HashSet<StateNode>()); //Add new world_pos - Hashset<float> pair to globalPathRegistry
                }
                globalPathRegistry[node.world_position].Add(node); //Add this node's orientation to Hashset corresponding to same node's world_pos
            }
            finishAngle = path_nodes[path_nodes.Count - 1].orientation;
        }
        return path_points;
    }

    private void ComputeAllPaths()
    {
        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();

        Vector3 start_pos_global = m_MapManager.startPositions[this.myCarIndex];
        start_pos_global = SnapToGridCenter(start_pos_global, cell_size); // Update to center of cell

        Vector3 goal_pos_global;

        // Calculate the starting angle of the car
        // Get the current Y rotation angle (in degrees)
        float currentAngle = m_Car.transform.rotation.eulerAngles.y;

        // Find the closest multiple of 30 degrees
        float closestMultipleOf30 = Mathf.Round(currentAngle / 30f) * 30f;

        // Ensure the angle is within 0 to 360 degrees
        float startAngle = closestMultipleOf30 % 360f;

        // If the result is negative, ensure it's within the positive range (0 to 360)
        if (startAngle < 0)
        {
            startAngle += 360f;
        }

        List<Vector3> new_path;
        float finishAngle;

        // Get the team for this vehicle
        string teamKey = gameManagerA2.GetGroup(this.gameObject);

        if (teamKey == "Free") // Not bakery
        {
            goal_pos_global = m_MapManager.targetPositions[this.myCarIndex];
            new_path = ComputePath(start_pos_global, goal_pos_global, startAngle, out finishAngle);
            list_paths.Add(new_path);
        }
        else //Bakery
        {
            // Get goals assigned to the team
            teamVehicles = gameManagerA2.GetGroupVehicles(this.gameObject); //Other vehicles in a Group with this vehicle
            List<MultiVehicleGoal> teamGoals = gameManagerA2.GetGoals(teamVehicles.First());

            if (teamGoals.Count == teamVehicles.Count) // same number of vehicles and goals
            {
                // Assign one goal per vehicle within the team
                int index = teamVehicles.IndexOf(this.gameObject);
                goal_pos_global = teamGoals[index].GetTargetObject().transform.position;
                new_path = ComputePath(start_pos_global, goal_pos_global, startAngle, out finishAngle);
                list_paths.Add(new_path);
            }

            else if (teamGoals.Count < teamVehicles.Count) // less goals than vehicles
            {
                // Assign one goal per vehicle within the team if index of vehicle < nb of goals
                int index = teamVehicles.IndexOf(this.gameObject);
                if (index < teamGoals.Count)
                {
                    goal_pos_global = teamGoals[index].GetTargetObject().transform.position;
                    new_path = ComputePath(start_pos_global, goal_pos_global, startAngle, out finishAngle);
                    list_paths.Add(new_path);
                }
            }
            else // more goals than vehicles
            {
                int index = teamVehicles.IndexOf(this.gameObject);
                while (index < teamGoals.Count) {
                    goal_pos_global = teamGoals[index].GetTargetObject().transform.position;
                    new_path = ComputePath(start_pos_global, goal_pos_global, startAngle, out finishAngle);
                    if (new_path.Count == 0) break;
                    list_paths.Add(new_path);
                    startAngle = finishAngle;
                    start_pos_global = goal_pos_global;
                    start_pos_global = SnapToGridCenter(start_pos_global, cell_size); // Update to center of cell
                    index += teamVehicles.Count;
                }
            }
        }
    }

}

