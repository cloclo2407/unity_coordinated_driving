using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Numerics;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
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
    public static int carCounter = 0; //This field belongs to the class/type, not to any specific object of the class

    //It is used to give an index to the specific car clone that has this script attached.
    public int myCarIndex; //This car's specific index in array of agents m_OtherCars (includes this car!)
    private int crazyCarIndex = 10; //For debugging when a specific car is acting crazy
    private float waiting_multiplier = 0f;
    private bool start_moving = false;
    public bool goal_reached = false;

    private CarController m_Car; // the car controller we want to use
    private BoxCollider carCollider; //unused BoxCollider
    private Rigidbody my_rigidbody;
    private MapManager m_MapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private List<MultiVehicleGoal> m_CurrentGoals;

    private ImprovePath m_improvePath;
    private Orca m_Orca;
    private Formation m_Formation;
    private Intersection m_Intersection;
    
    private List<StateNode> path_of_nodes = new List<StateNode>();
    public List<Vector3> path_of_points = new List<Vector3>();

    public int currentPathIndex = 1;
    public Vector3 target_position;
    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public float k_p = 1f;
    public float k_d = 2f;
    private bool isStuck = false;
    private int timeStuck = 0;
    private float reverseDuration = 0;

    public bool drawTargets;
    public bool drawAllCars;
    public bool drawTeamCars;

    public bool IsBeingFollowed = false; // indicates if you're being followed by another car
    public CarController followingCar = null; // contains that is following you
    public CarController carToFollow = null; // contains the car you are following

    //For driving:
    private float waypoint_margin = 4f; //Math.Clamp(my_rigidbody.linearVelocity.magnitude, 5f, 15f); //6.5f; //Serves as a means of checking if we're close enough to goal/ next waypoint
    private float speed_limit = 3.5f;
    private float max_scan_distance = 7.5f; // Testing a variable scan distance
    float safeFollowDistance = 6f; // Minimum distance to keep behind the car we're following
    public float distToPoint = 4f; // min distance to go to the next point in path
    public bool hasToStop; // for intersection

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

        m_improvePath = new ImprovePath();
        m_Formation = new Formation();
        m_Intersection = new Intersection();
        
        m_MapManager = FindFirstObjectByType<MapManager>();
        float cell_size = 3f;
        Vector3 cell_scale = Vector3.one * cell_size;
        m_ObstacleMap = ObstacleMap.Initialize(m_MapManager, new List<GameObject>(), cell_scale);
        m_ObstacleMap.margin = Vector3.one * 2f; // (Changing cell margins, do they work?)

        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        m_CurrentGoals = gameManagerA2.GetGoals(this.gameObject); // This car's goal.
        teamVehicles = gameManagerA2.GetGroupVehicles(this.gameObject); //Other vehicles in a Group with this vehicle
        m_OtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles

        m_Orca = new Orca(m_Car, my_rigidbody, m_OtherCars);

        // Note that this array will have "holes" when objects are destroyed. For initial planning they should work.
        // If you don't like the "holes", you can re-fetch this during fixed update.
        // Equivalent ways to find all the targets in the scene
        targetObjects = m_CurrentGoals.Select(goal => goal.GetTargetObject()).ToList(); //targetObjects is a list of one element for some reason
        // You can also fetch other types of objects using tags, assuming the objects you are looking for HAVE tags :).
        // Feel free to refer to any examples from previous assignments.

        ////////////////////////// Plan your path here
        myCarIndex = carCounter % m_MapManager.startPositions.Count; //Index of this specific car
        carCounter++; //myCarIndex is used to find the specific start_pos and goal_pos of this car

        Vector3 start_pos_global = m_MapManager.startPositions[this.myCarIndex];
        Vector3 goal_pos_global = m_MapManager.targetPositions[this.myCarIndex];

        start_pos_global = SnapToGridCenter(start_pos_global, cell_size); // Update to center of cell

        // Calculate the starting angle of the car
        // Get the current Y rotation angle (in degrees)
        float currentAngle = m_Car.transform.rotation.eulerAngles.y;

        // Find the closest multiple of 30 degrees
        float closestMultipleOf30 = Mathf.Round(currentAngle / 30f) * 30f;

        // Ensure the angle is within 0 to 360 degrees
        float resultAngle = closestMultipleOf30 % 360f;

        // If the result is negative, ensure it's within the positive range (0 to 360)
        if (resultAngle < 0)
        {
            resultAngle += 360f;
        }
        
        if (crazyCarIndex == myCarIndex) Debug.Log("currentAngle: "+currentAngle+", closestMultOf30: "+closestMultipleOf30+", resultAngle: "+resultAngle);

        PriorityQueue Q = new PriorityQueue();
        StateNode.crazyCarIndex = crazyCarIndex; //for debugging
        Dictionary<Vector3Int, StateNode> visited_nodes = new Dictionary<Vector3Int, StateNode>();
        StateNode start_node = new StateNode(start_pos_global, resultAngle, goal_pos_global, null, m_MapManager, m_ObstacleMap, myCarIndex);
        Q.Enqueue(start_node);

        while (Q.Count != 0)
        {
            var current_node = Q.Dequeue();
            if (!visited_nodes.ContainsKey(current_node.cell_position))
            {
                visited_nodes.Add(current_node.cell_position, current_node);
            }

            if (Vector3.Distance(current_node.world_position, goal_pos_global) <=
                this.waypoint_margin) //We have reached the goal, time to create the path
            {
                current_node.fillPaths(this.path_of_nodes, this.path_of_points);
                //Now the path_of_nodes and path_of_points are ready to be executed. We can either use StateNodes or Vector3s in the controller.
                break; // BREAK OUT OF WHILE LOOP, WE'RE DONE HERE
            }

            //else we keep looking:
            List<StateNode> new_nodes = current_node.makeChildNodes(visited_nodes, Q, goal_pos_global, m_MapManager, m_ObstacleMap, cell_scale.z, "car");
            foreach (StateNode n in new_nodes)
            {
                Q.Enqueue(n);
            } //Add all new nodes to the queue
        }

        //Add all visited waypoint cells to hashset keeping track for other runs of A* for other cars (do this before smoothing):
        foreach (StateNode node in path_of_nodes)
        {
            StateNode.used_waypoints.Add(node.cell_position);
        }

        // Plot your path to see if it makes sense. Note that path can only be seen in "Scene" window, not "Game" window
        for (int i = 0; i < path_of_points.Count - 1; i++) {
            // Debug.drawline draws a line between a start point and end point IN GLOBAL COORDS!
            //if (crazyCarIndex == myCarIndex) Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f);
            Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f);
        }   

        //Catmull-Rom:
        //path_of_points = m_improvePath.SmoothSplineCatmullRom(path_of_points, 5);
        //path_of_points = m_improvePath.simplifyPath(path_of_points, 0.1f); //Disabled because it helps ORCA to have closer waypoints
        //for (int j = 0; j < path_of_points.Count-1; j++)
        //{ Debug.DrawLine(path_of_points[j] + Vector3.up, path_of_points[j+1] + Vector3.up, Color.yellow, 1000f); }

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
        if (goal_reached == true) //Make the car stop after reaching goal.
        {
            if (my_rigidbody.linearVelocity.magnitude > 0.005f) m_Car.Move(0f, 0f, 0f, 100f);
        }
        
        else if (start_moving == true && path_of_points.Count != 0 && currentPathIndex < path_of_points.Count)
        {
            //Checks if we have any driving left to do
            //Check if we need to evade other cars with Orca:
            //m_Orca.UpdateNeighboringAgents();
            Vector3 new_velocity = Vector3.zero;
            //if (m_Orca.NeedOrca()) new_velocity = m_Orca.EvadeCollisionWithORCA(); //We have other agents close by, use ORCA
            //else new_velocity = Vector3.zero;
            DriveAndRecover(new_velocity); //follow path, recover if stuck
        }
    }

    private void OnDrawGizmos() // This method is called by the Unity Editor everytime FixedUpdate() runs.
    {
        // It must not be called by us during runtime, that will raise an error
        //Handles.color = Color.red; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
        //Handles.DrawWireDisc(transform.position, Vector3.up, m_Orca.neighbor_radius);

        Debug.DrawLine(transform.position + Vector3.up * 2f, target_position + Vector3.up * 2f, Color.red);  // Shows where we're aiming to follow

        if (hasToStop)
        {
            Handles.color = Color.blue; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f); // draw a blue circle around a car that is stopping for intersection
        }

    }

    //Follow waypoints and do collision recovery
    private void DriveAndRecover(Vector3 orca_velocity) 
    { 
        //Detect obstacles
        UpdateRaycast();

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
            m_Formation.LineFormation(m_Car, m_OtherCars, target_position); // Check for car to follow

            if (hasToStop) // If I have to stop for intersection, break
            {
                m_Car.Move(0f, 0f, 100f, 1000f);
            }

            else
            {
                if (carToFollow != null) // I'm following a car
                {
                    target_position = carToFollow.transform.position;
                    target_position = target_position - carToFollow.transform.forward * safeFollowDistance; // Aim for behind the car

                    if (currentPathIndex != path_of_points.Count - 1) distToPoint = 4f; // Can validate point from further if you're following a car
                }

                /*else if (orca_velocity != Vector3.zero)
                {
                    target_position = transform.position + orca_velocity;
                    target_velocity = orca_velocity;
                }**/
   
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
                
                // Break if I'm too close to the car I follow, help inserting to keep a distance between cars
                if (carToFollow != null && Vector3.Distance(transform.position, carToFollow.transform.position) < safeFollowDistance) 
                {
                    m_Car.Move(0f, 0f, 100f, 100f);
                    timeStuck = 0;
                }

                //Drive
                else m_Car.Move(steering, acceleration, acceleration, 0f);              
            }

            // Update currentPathIndex
            if (Vector3.Distance(path_of_points[currentPathIndex], transform.position) < distToPoint)
            {
                currentPathIndex++;          

                if (currentPathIndex == path_of_points.Count - 1) {distToPoint = 1; } //Changing distToPoint to be smaller when next waypoint is the goal

                // Can set a bigger distance to help the beginning
                else if (currentPathIndex < 4)
                {
                    distToPoint = 4f;
                }
                else { distToPoint = 4f; } //reset

                if (currentPathIndex == path_of_points.Count) goal_reached = true;  
            }

            // If you're barely moving it means you may be stuckstuck
            if (my_rigidbody.linearVelocity.magnitude < 0.1f)
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
                m_Car.Move(0f, 0f, 100f, 1000f);
            }
        }
    }

    // Not used for now
    private int FindNearestForwardIndex(Vector3 currentPosition, int startIndex, float searchRadius)
    {
        int nearestIndex = startIndex;
        float smallestDistance = float.MaxValue;
        Vector3 carForward = transform.forward;

        for (int i = startIndex; i < path_of_points.Count; i++)
        {
            Vector3 toPathPoint = path_of_points[i] - currentPosition;
            float distance = toPathPoint.magnitude;

            // Consider points that are somewhat in front of the car
            if (Vector3.Dot(toPathPoint.normalized, carForward) > 0f)
            {
                if (distance < smallestDistance && distance < searchRadius)
                {
                    smallestDistance = distance;
                    nearestIndex = i;
                }
            }
        }
        return Mathf.Max(nearestIndex, currentPathIndex);
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

        // Draw Raycasts in Blue
        /* Debug.DrawRay(transform.position, directionRight * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionLeft * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionBackRight * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionBackLeft * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionBack * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, transform.forward * maxRangeClose, Color.blue); */
    }

    // Function to round starting coordinates to the center of the cell
    Vector3 SnapToGridCenter(Vector3 position, float cell_scale)
    {
        return new Vector3(
            Mathf.Round(position.x / cell_scale) * cell_scale + cell_scale / 2f,
            position.y, // Keep the original Y value
            Mathf.Round(position.z / cell_scale) * cell_scale + cell_scale / 2f
        );
    }

}

