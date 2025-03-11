using Scripts.Vehicle;
using UnityEngine;
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

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : MonoBehaviour
{
    public static int carCounter = 0; //This field belongs to the class/type, not to any specific object of the class

    //It is used to give an index to the specific car clone that has this script attached.
    public int myCarIndex; //This car's specific index in array of agents m_OtherCars (includes this car!)
    private int crazyCarIndex = 1; //For debugging when a specific car is acting crazy
    private float waiting_multiplier = 0f;
    private bool start_moving = false;
    private bool goal_reached = false;

    private DroneController m_Drone; // the car controller we want to use
    private BoxCollider droneCollider; //unused BoxCollider
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

    public bool IsBeingFollowed = false;
    public GameObject followingCar = null;
    public GameObject carToFollow = null;

    //For driving:
    private float waypoint_margin = 3f; //Math.Clamp(my_rigidbody.linearVelocity.magnitude, 5f, 15f); //6.5f; //Serves as a means of checking if we're close enough to goal/ next waypoint
    private float speed_limit = 3.5f;
    private float max_scan_distance = 7.5f; // Testing a variable scan distance
    float safeFollowDistance = 4f; // Minimum distance to keep behind the car we're following
    public float distToPoint = 6f;
    public bool hasToStop;

    //private bool obstacles_close = false;
    private List<Vector3> raycast_hit_positions = new List<Vector3>();

    private float obstacle_avoiding_steering = 0f; //Addition to steering input calculated via raycasts of environment

    //private float obstacle_avoiding_accel = 0f;    //in order to steer away from obstacles, but remain on path
    private float obstacle_avoiding_steering_limit = 40f; //TODO
    //private float obstacle_avoiding_accel_limit = 0f; //TODO
    //private bool open_area = false; //Sets parameters for driving in open areas, like TerrainA. TODO
    //private bool cluttered_area = true; //Sets parameters for driving in cluttered areas, like TerrainD. TODO

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
        m_Drone = GetComponent<DroneController>();
        my_rigidbody = GetComponent<Rigidbody>();

        m_improvePath = new ImprovePath();
        //m_Formation = new Formation();
        //m_Intersection = new Intersection();
        
        m_MapManager = FindFirstObjectByType<MapManager>();
        Vector3 cell_scale = Vector3.one * 3f;
        m_ObstacleMap = ObstacleMap.Initialize(m_MapManager, new List<GameObject>(), cell_scale);
        m_ObstacleMap.margin = Vector3.one * 2f; // (Changing cell margins, do they work?)

        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        m_CurrentGoals = gameManagerA2.GetGoals(this.gameObject); // This car's goal.
        teamVehicles = gameManagerA2.GetGroupVehicles(this.gameObject); //Other vehicles in a Group with this vehicle
        m_OtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles

        m_Orca = new Orca(this.gameObject, my_rigidbody, m_OtherCars);

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

        PriorityQueue Q = new PriorityQueue();
        Dictionary<Vector3Int, StateNode> visited_nodes = new Dictionary<Vector3Int, StateNode>();
        StateNode start_node = new StateNode(start_pos_global, 0f, goal_pos_global, null, m_MapManager, m_ObstacleMap);
        Q.Enqueue(start_node);

        while (Q.Count != 0)
        {
            var current_node = Q.Dequeue();
            visited_nodes.Add(current_node.cell_position, current_node);

            if (Vector3.Distance(current_node.world_position, goal_pos_global) <=
                this.waypoint_margin) //We have reached the goal, time to create the path
            {
                current_node.fillPaths(this.path_of_nodes, this.path_of_points);
                //Now the path_of_nodes and path_of_points are ready to be executed. We can either use StateNodes or Vector3s in the controller.
                break; // BREAK OUT OF WHILE LOOP, WE'RE DONE HERE
            }

            //else we keep looking:
            List<StateNode> new_nodes = current_node.makeChildNodes(visited_nodes, Q, goal_pos_global, m_MapManager, m_ObstacleMap, cell_scale.z, "drone");
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
            Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f);
        }   

        //Catmull-Rom:
        path_of_points = m_improvePath.SmoothSplineCatmullRom(path_of_points, 5);
        //path_of_points = m_improvePath.simplifyPath(path_of_points, 0.1f); //Disabled because it helps ORCA to have closer waypoints
        //for (int j = 0; j < path_of_points.Count-1; j++)
        //{ Debug.DrawLine(path_of_points[j] + Vector3.up, path_of_points[j+1] + Vector3.up, Color.yellow, 1000f); }

        target_position = path_of_points[0];
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
            if (my_rigidbody.linearVelocity.magnitude > 0.05f) StopDrone();
        }
        
        else if (start_moving == true && path_of_points.Count != 0 && currentPathIndex < path_of_points.Count)
        {
            //Checks if we have any driving left to do
            //Check if we need to evade other cars with Orca:
            m_Orca.UpdateNeighboringAgents();
            Vector3 new_velocity;
            if (m_Orca.NeedOrca()) new_velocity = m_Orca.EvadeCollisionWithORCA("drone"); //We have other agents close by, use ORCA
            else new_velocity = Vector3.zero;
            DriveAndRecover(new_velocity); //follow path, recover if stuck
        }
    }

    private void OnDrawGizmos() // This method is called by the Unity Editor everytime FixedUpdate() runs.
    {
        // It must not be called by us during runtime, that will raise an error
        Handles.color = Color.red; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
        Handles.DrawWireDisc(transform.position, Vector3.up, m_Orca.neighbor_radius);

        Debug.DrawLine(transform.position + Vector3.up * 2f, target_position + Vector3.up * 2f, Color.red);  // Shows where we're aiming to follow

        if (hasToStop)
        {
            Handles.color = Color.blue; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
            Handles.DrawWireDisc(transform.position, Vector3.up, 4f);
        }

    }

    private void DriveAndRecover(Vector3 orca_velocity) 
    { //Follow waypoints and do collision recovery

        if (orca_velocity == Vector3.zero)
        { target_position = path_of_points[currentPathIndex]; }

        else 
        { target_position = transform.position + orca_velocity; }
        
        (float h_input, float v_input) = droneController();
        m_Drone.Move(h_input, v_input);
        
        if (Vector3.Distance(path_of_points[currentPathIndex], transform.position) < distToPoint)
        {
            currentPathIndex++;
            if (currentPathIndex == path_of_points.Count - 1) {distToPoint = 1; } //Changing distToPoint to be smaller when next waypoint is the goal
            else {distToPoint = 4f; } //reset
            if (currentPathIndex == path_of_points.Count) goal_reached = true;  
        }

        Debug.DrawLine(transform.position, transform.position + orca_velocity, Color.black); //Draws white line if we have nonzero orca velocity            
    }

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

    private void UnStuck()
    {
        float h_in = 0f;
        float v_in = 0f;
        
        // TODO WHAT ABOUT obsStraightRightClose AND obsStraightLeftClose???????? ADD THEM! Where else in the program are they needed?
        
        if (obsRightClose) h_in += -1f;
        if (obsLeftClose) h_in += 1f;
        if (obsStraightClose) v_in += -1f;
        if (obsBackClose) v_in += 1f;
        if (obsBackRightClose) h_in += -1f; v_in += 1f;
        if (obsBackLeftClose) h_in += 1f; v_in += 1f;
        
        m_Drone.Move(h_in, v_in);
    }

    private void StopDrone()
    { //Method makes drone stop moving
        
        // TODO NEEDS TESTING!!!!

        float h_input = 0f;
        float v_input = 0f;
        
        if (my_rigidbody.linearVelocity.x > 0.005f) h_input = -1f;
        else h_input = 1f;
        
        if (my_rigidbody.linearVelocity.z > 0.005f) v_input = -1f;
        else v_input = 1f;
            
        m_Drone.Move(h_input, v_input);
    }
    private (float, float) droneController()
    {
        //Calculate vector to target position
        Vector3 toTarget = target_position - transform.position;
        toTarget.y = 0f;
        
        // Get angle to target waypoint
        float signedAngle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
        float angleRad = signedAngle * Mathf.Deg2Rad; //convert angle to radians

        // Calculate movements in horizontal/ vertical directions
        float horizontal = Mathf.Sin(angleRad);
        float vertical = Mathf.Cos(angleRad);

        // Debug velocity vector
        Debug.DrawLine(transform.position, transform.position + my_rigidbody.linearVelocity, Color.yellow);
        if (my_rigidbody.linearVelocity.magnitude >= speed_limit)
        {
            Vector3 deaccelerationVector = -my_rigidbody.linearVelocity.normalized;
            float excessSpeed = my_rigidbody.linearVelocity.magnitude - speed_limit;
            float decelerationFactor = Mathf.Clamp01((excessSpeed + 1) / speed_limit);
            //m_Drone.Move(deaccelerationVector.x * decelerationFactor, deaccelerationVector.z * decelerationFactor);
            //Debug.DrawLine(transform.position, transform.position + deaccelerationVector, Color.green);
            //Debug.Log($"Speed too high! Reducing with factor: {decelerationFactor}, DecelerationForce: {deaccelerationVector}");
            return (deaccelerationVector.x * decelerationFactor, deaccelerationVector.z*decelerationFactor);
        }
        else
        {
            //m_Drone.Move(horizontal * droneSpeedCap, vertical * droneSpeedCap);
            //m_Drone.Move(horizontal, vertical);
            return (horizontal, vertical);
        }

        //Debug.Log($"Angle: {signedAngle}° | H: {horizontal} | V: {vertical} " +
        //          $"| Speed: {my_rigidbody.linearVelocity.magnitude} | Limit: {speedLimit}");
    }

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

}

