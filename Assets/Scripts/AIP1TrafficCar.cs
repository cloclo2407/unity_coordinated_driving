using System.Collections.Generic;
using System.Collections;
using System.Linq;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Map;
using Scripts.Game;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : MonoBehaviour
{
    public static int carCounter = 0; //This field belongs to the class/type, not to any specific object of the class
    //It is used to give an index to the specific car clone that has this script attached.
    private int myCarIndex; //This car's specific index
    
    private CarController m_Car; // the car controller we want to use
    private BoxCollider carCollider; //unused BoxCollider
    private Rigidbody my_rigidbody;
    private MapManager m_MapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private List<MultiVehicleGoal> m_CurrentGoals;
    
    private List<StateNode> path_of_nodes = new List<StateNode>();
    private List<Vector3> path_of_points = new List<Vector3>();
    private List<Vector3> smooth_path_of_points = new List<Vector3>();
    
    public bool drawTargets;
    public bool drawAllCars;
    public bool drawTeamCars;
    
    //Here is a new comment, wowowowo

    //For driving:
    private float waypoint_margin = 3f; //Math.Clamp(my_rigidbody.linearVelocity.magnitude, 5f, 15f); //6.5f; //Serves as a means of checking if we're close enough to goal/ next waypoint
    private float speed_limit = 3.5f;
    private float max_scan_distance = 7.5f; // Testing a variable scan distance
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

    private void Start()
    {
        m_Car = GetComponent<CarController>();
        my_rigidbody = GetComponent<Rigidbody>();

        m_MapManager = FindFirstObjectByType<MapManager>();
        Vector3 cell_scale = Vector3.one * 3f;
        m_ObstacleMap = ObstacleMap.Initialize(m_MapManager, new List<GameObject>(), cell_scale);
        m_ObstacleMap.margin = Vector3.one * 2f; // (Changing cell margins, do they work?)

        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        m_CurrentGoals = gameManagerA2.GetGoals(this.gameObject); // This car's goal.
        teamVehicles = gameManagerA2.GetGroupVehicles(this.gameObject); //Other vehicles in a Group with this vehicle
        m_OtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles

        // Note that this array will have "holes" when objects are destroyed. For initial planning they should work.
        // If you don't like the "holes", you can re-fetch this during fixed update.
        // Equivalent ways to find all the targets in the scene
        targetObjects = m_CurrentGoals.Select(goal => goal.GetTargetObject()).ToList(); //targetObjects is a list of one element for some reason
        // You can also fetch other types of objects using tags, assuming the objects you are looking for HAVE tags :).
        // Feel free to refer to any examples from previous assignments.

        ////////////////////////// Plan your path here
        myCarIndex = carCounter % m_MapManager.startPositions.Count; //Index of this specific car
        carCounter++; //myCarIndex is used to find the specific start_pos and goal_pos of this car
        Debug.Log("myCarIndex: " + myCarIndex + ", carCounter: " + carCounter + ", targetObjects.Count: " + targetObjects.Count);
        Vector3 start_pos_global = m_MapManager.startPositions[this.myCarIndex];
        Vector3 goal_pos_global = m_MapManager.targetPositions[this.myCarIndex];

        Debug.Log("start_pos: " + start_pos_global);
        Debug.Log("goal_pos: " + goal_pos_global);

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
            List<StateNode> new_nodes = current_node.makeChildNodes(visited_nodes, Q, m_MapManager, m_ObstacleMap, cell_scale.z, "car");
            foreach (StateNode n in new_nodes)
            {
                Q.Enqueue(n);
            } //Add all new nodes to the queue
        }

        Debug.Log("Stopped looking for path (left the while-loop)");

        //Add all visited waypoint cells to hashset keeping track for other runs of A* for other cars (do this before smoothing):
        foreach (StateNode node in path_of_nodes)
        {
            StateNode.used_waypoints.Add(node.cell_position);
        }

        // Plot your path to see if it makes sense. Note that path can only be seen in "Scene" window, not "Game" window
        for (int i = 0; i < path_of_points.Count - 1; i++) // Debug.drawline draws a line between a start point and end point IN GLOBAL COORDS!
        { Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f); } 

        //////////////////////////Catmull-Rom:
        SmoothSplineCatmullRom(path_of_points, 5);
        //for (int j = 0; j < smooth_path_of_points.Count-1; j++)
        //{ Debug.DrawLine(smooth_path_of_points[j] + Vector3.up, smooth_path_of_points[j+1] + Vector3.up, Color.yellow, 1000f); }

        Debug.Log("Path of points contains :"+ path_of_points.Count + "points");
        Debug.Log("Smooth path contains :"+ smooth_path_of_points.Count + "points");
    }
    
    public void SmoothSplineCatmullRom(List<Vector3> originalPath, int subdivisionsPerSegment = 5)
    {
        if (originalPath.Count < 2)
            return;
        this.smooth_path_of_points.Clear();
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

    /*
    private void Update()
    {
        if(drawTargets)
        {
            foreach (var item in targetObjects)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.red);
            }
        }

        if(drawTeamCars)
        {
            foreach (var item in teamVehicles)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.blue);
            }
        }

        if(drawAllCars)
        {
            foreach (var item in m_OtherCars)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.yellow);
            }
        }
        //Debug.DrawLine(Vector3.zero, new Vector3(1, 0, 0), Color.red);
    }
    */


    private void FixedUpdate()
    {   // Feel free to refer to any examples from previous assignments.

        if (smooth_path_of_points.Count == 0)
        { } //Goal reached, do nothing
        else
        {   //Execute your path here
            Vector3 goal_pos_global = targetObjects[0].transform.position; //targetObjects is a list of one element for some reason
            Vector3 car_pos_global = transform.position;
            Vector3 goal_vector = car_pos_global - goal_pos_global;
            var distance_from_goal = goal_vector.magnitude;
            var target_position = smooth_path_of_points[0];

            //Dynamic obstacle avoidance with RAYCASTS
            raycast_hit_positions.Clear();
            float scan_distance;
            //I'm working with a droplet-shape of rays. I want the front rays to be longer than the ones on the sides of the car.
            //Mapping function f(x) = 1-abs(x)/45. x is our angle i, f(x) is our scan distance.
            for (float i = -80f; i <= 80f; i += 5f)
            {
                // 90/5=18 rays
                scan_distance = max_scan_distance - max_scan_distance * (Mathf.Abs(i) / 80f);
                Vector3 direction_i =
                    Vector3.Normalize(Quaternion.Euler(0, i, 0) *
                                      transform
                                          .forward); //Quat.Euler(0,i,0)*transf.fwd rotates transf.fwd i degrees around y-axis
                Debug.DrawLine(transform.position, transform.position + (direction_i * scan_distance), Color.blue);
                RaycastHit hit_object; //declare hit-object
                if (Physics.Raycast(transform.position, direction_i, out hit_object, scan_distance))
                {
                    //obstacle detected close to vehicle, save the position where raycast hit it
                    raycast_hit_positions.Add(hit_object
                        .point); //adding point of impact (in global coords) where ray hit the obstacle
                }
            }

            if (raycast_hit_positions.Count > 0) //We are currently looking at obstacles with our raycasts
            {
                waypoint_margin = 4f; //Increasing waypoint margin while maneuvering away from obstacle to not lose path
                foreach (Vector3 hit_pos in raycast_hit_positions)
                {
                    //Calculate steering input to obstacle, and then add its negative counterpart (scaled down) to steering inputs later
                    if (obstacle_avoiding_steering < obstacle_avoiding_steering_limit)
                    {
                        float collide_with_obstacle_steering;
                        float collide_with_obstacle_accel; //currently unused
                        (collide_with_obstacle_steering, collide_with_obstacle_accel) = ControlsTowardsPoint(hit_pos);
                        obstacle_avoiding_steering += -collide_with_obstacle_steering;
                        obstacle_avoiding_steering = Mathf.Clamp(obstacle_avoiding_steering,
                            -1 * obstacle_avoiding_steering_limit, obstacle_avoiding_steering_limit);
                    }
                }
            }
            else //No obstacles nearby
            {
                obstacle_avoiding_steering = 0f;
                waypoint_margin =
                    3f; // We only decrease waypoint_margin IF car_is_perpendicular = false AND we do not spot obstacles w. raycast.
            }

            //Driving
            if (Vector3.Magnitude(car_pos_global - smooth_path_of_points[0]) < waypoint_margin)
            {
                // we have reached the current target position
                //Debug.Log("Reached next target position!");
                smooth_path_of_points.RemoveAt(0); //remove index 0 so we can go to next target
            }
            else // We are driving to the next waypoint
            {
                (steering, acceleration) = ControlsTowardsPoint(target_position);
                acceleration =
                    (my_rigidbody.linearVelocity.magnitude > speed_limit)
                        ? -1f
                        : acceleration; //?: "ternary operator". Assigns accel the value to left of : if condition in front of ? is true, otherwise assigns accel = value to right of :
                Debug.DrawLine(transform.position, target_position + Vector3.up, Color.red);
                m_Car.Move(steering + obstacle_avoiding_steering, acceleration, acceleration, 0f);
            }
        }
    }

    private (float steering, float acceleration) ControlsTowardsPoint(Vector3 avg_pos)
    {
        Vector3 direction = (avg_pos - transform.position).normalized;

        bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
        bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

        float steering = 0f;
        float acceleration = 0;

        if (is_to_the_right && is_to_the_front)
        {
            steering = 1f;
            acceleration = 1f;
        }
        else if (is_to_the_right && !is_to_the_front)
        {
            steering = -1f;
            acceleration = -1f;
        }
        else if (!is_to_the_right && is_to_the_front)
        {
            steering = -1f;
            acceleration = 1f;
        }
        else if (!is_to_the_right && !is_to_the_front)
        {
            steering = 1f;
            acceleration = -1f;
        }

        float alpha = Mathf.Asin(Vector3.Dot(direction, transform.right));
        if (is_to_the_front && Mathf.Abs(alpha) < 1f)
        {
            steering = alpha;
        }
        
        return (steering, acceleration);
    }
}

