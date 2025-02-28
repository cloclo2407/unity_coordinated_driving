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
    private int myCarIndex; //This car's specific index

    private CarController m_Car; // the car controller we want to use
    private BoxCollider carCollider; //unused BoxCollider
    private Rigidbody my_rigidbody;
    private MapManager m_MapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private List<MultiVehicleGoal> m_CurrentGoals;
    private ImprovePath improvePath;


    private List<StateNode> path_of_nodes = new List<StateNode>();
    private List<Vector3> path_of_points = new List<Vector3>();

    private int currentPathIndex = 1;
    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public Vector3 desired_velocity;
    public float k_p = 1f;
    public float k_d = 2f;
    private bool isStuck = false;
    private int timeStuck = 0;
    private float reverseDuration = 0;
    private bool checkNewPoint = true;


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

    //ORCA-parameters:
    private float neighbor_radius = 16f;
    private List<GameObject> neighbor_agents = new List<GameObject>();
    private float timeHorizon = 10f; //tau in the report
    private List<Tuple<Vector3, Vector3>> orca_constraints = new List<Tuple<Vector3, Vector3>>();
    private int max_considered_neighbors = 5;

    private void Start()
    {
        m_Car = GetComponent<CarController>();
        my_rigidbody = GetComponent<Rigidbody>();
        improvePath = new ImprovePath();


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
        targetObjects =
            m_CurrentGoals.Select(goal => goal.GetTargetObject())
                .ToList(); //targetObjects is a list of one element for some reason
        // You can also fetch other types of objects using tags, assuming the objects you are looking for HAVE tags :).
        // Feel free to refer to any examples from previous assignments.

        ////////////////////////// Plan your path here
        myCarIndex = carCounter % m_MapManager.startPositions.Count; //Index of this specific car
        carCounter++; //myCarIndex is used to find the specific start_pos and goal_pos of this car
        Debug.Log("myCarIndex: " + myCarIndex + ", carCounter: " + carCounter + ", targetObjects.Count: " +
                  targetObjects.Count);
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
            List<StateNode> new_nodes =
                current_node.makeChildNodes(visited_nodes, Q, m_MapManager, m_ObstacleMap, cell_scale.z, "car");
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
        /* for (int i = 0;
             i < path_of_points.Count - 1;
             i++) // Debug.drawline draws a line between a start point and end point IN GLOBAL COORDS!
        {
            Debug.DrawLine(path_of_points[i] + Vector3.up, path_of_points[i + 1] + Vector3.up, Color.magenta, 1000f);
        }*/

        //////////////////////////Catmull-Rom:
        path_of_points = improvePath.SmoothSplineCatmullRom(path_of_points, 5);
        //path_of_points = improvePath.simplifyPath(path_of_points, 0.1f); //Disabled because it helps ORCA to have closer waypoints
        //for (int j = 0; j < path_of_points.Count-1; j++)
        //{ Debug.DrawLine(path_of_points[j] + Vector3.up, path_of_points[j+1] + Vector3.up, Color.yellow, 1000f); }

        Debug.Log("Path of points contains :" + path_of_points.Count + "points");
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
    {
        // Feel free to refer to any examples from previous assignments.

        if (path_of_points.Count != 0 && currentPathIndex < path_of_points.Count)
        {
            //Checks if we have any driving left to do

            //Check if we need to evade other cars with Orca:
            UpdateNeighboringAgents();
            if (neighbor_agents.Count() != 0) EvadeCollisionWithORCA(); //We have other agents close by, use ORCA
            else DriveAndRecover(Vector3.zero); //follow path, recover if stuck
        }
    }

    private void OnDrawGizmos() // This method is called by the Unity Editor everytime FixedUpdate() runs.
    {
        // It must not be called by us during runtime, that will raise an error
        Handles.color = Color.red; //Handles are used for debugging in scene view with Unity Editor, never in the game runtime
        Handles.DrawWireDisc(transform.position, Vector3.up, neighbor_radius);
    }

    private void EvadeCollisionWithORCA()
    {
        orca_constraints.Clear(); //Reset ORCA constraints
        Vector3 v_A = my_rigidbody.linearVelocity; //Let this be agent A and other be agent B
        Vector3 pos_A = transform.position;
        float car_radius = 2.2f; //We approximate the car as a circular robot with radius 2.2 based on the prefab model

        foreach (GameObject other_agent in neighbor_agents)
        {
            //Check if we are on collision course with other agent. Compute and store constraint if that is the case.
            var rigidbody_B = other_agent.GetComponent<Rigidbody>();
            Vector3 v_B = rigidbody_B.linearVelocity;
            Vector3 pos_B = other_agent.transform.position;
            Vector3 relative_velocity = (v_A - v_B); //Is this the correct way of calculating relative_vel
            Vector3 relative_position = (pos_B - pos_A); // and relative_pos?

            float theta =
                Mathf.Asin((car_radius + car_radius) /
                           relative_position.magnitude); //Angle defining if relative_vel is in velocity obstacle
            Vector3 max_considered_velocity = relative_velocity + relative_position / timeHorizon;

            float relative_vectors_signed_angle = Vector3.SignedAngle(relative_position, relative_velocity, Vector3.up);
            //angle <0 means relative_vel is rotated counterclockwise from relative_pos by angle degrees
            //angle >0 means relative_vel is rotated clockwise from relative_pos by angle degrees

            if (Mathf.Abs(relative_vectors_signed_angle) < theta &&
                relative_velocity.magnitude < max_considered_velocity.magnitude)
            {
                //We have a collision to avoid. Let's compute and store the constraint for the new velocity

                if (relative_vectors_signed_angle < 0) //relative_vel in left triangle of VO
                {
                    Vector3 left_bound_vector = Quaternion.Euler(0, theta, 0) * relative_position;
                    Vector3 projected_relative_velocity = Vector3.Project(relative_velocity, left_bound_vector);
                    Vector3 smallest_change_to_avoid_collision = projected_relative_velocity - relative_velocity;
                    Vector3 point_defining_constraint = v_A + 0.5f * smallest_change_to_avoid_collision;
                    Vector3 norm_defining_constraint = Vector3.Normalize(smallest_change_to_avoid_collision);
                    Tuple<Vector3, Vector3> new_constraint =
                        new Tuple<Vector3, Vector3>(point_defining_constraint, norm_defining_constraint);
                    orca_constraints.Add(new_constraint);
                }

                //NOTE: left_bound_vector has the correct direction of the left bound of the left triangle, but has
                //the wrong magnitude, same mag as relative_pos. Since we are only using left_bound_vector as a vector
                //to project relative_vel onto, its magnitude doesn't matter as long as the direction is correct.
                // Proof: let * be the scalar product, k be a scalar and u,v be vectors:
                //proj_u_on_v = (u*v / v*v)v. If v is scaled by scalar k, its magnitude is changed but direction is not:
                //proj_u_on_kv = (u*(kv) / kv*kv)(kv) = (k/k^2)(u*v/v*v)(kv) = (k^2/k^2)(u*v/v*v)v = (u*v/v*v)v

                else //signed_angle >= 0, relative_vel in right triangle of VO
                {
                    Vector3 right_bound_vector = Quaternion.Euler(0, -theta, 0) * relative_position;
                    Vector3 projected_relative_velocity = Vector3.Project(relative_velocity, right_bound_vector);
                    Vector3 smallest_change_to_avoid_collision = projected_relative_velocity - relative_velocity;
                    Vector3 point_defining_constraint = v_A + 0.5f * smallest_change_to_avoid_collision;
                    Vector3 norm_defining_constraint = Vector3.Normalize(smallest_change_to_avoid_collision);
                    Tuple<Vector3, Vector3> new_constraint =
                        new Tuple<Vector3, Vector3>(point_defining_constraint, norm_defining_constraint);
                    orca_constraints.Add(new_constraint);
                }
            }
        }

        if (orca_constraints.Count() > 0) //We have an optimization problem to solve
        {
            //Draw constraint-lines and corrsponding normals for debugging:
            foreach (Tuple<Vector3, Vector3> constraint in orca_constraints)
            {
                //Get vector parallel to line. constraint.Item1 is the point on the line, constraint.Item2 is the Vector3 normal vector to line
                Vector2 perpendicular2D = new Vector2(constraint.Item2.x, constraint.Item2.z);
                Vector2 parallel2D = Vector2.Perpendicular(perpendicular2D);
                Vector3 parallel_vector = new Vector3(parallel2D.x, 0, parallel2D.y);
                Vector3 line_starting_point = transform.position + constraint.Item1;
                
                //Draw normal to constraint-line
                Debug.DrawLine(line_starting_point, line_starting_point+constraint.Item2*5f, Color.cyan);
                //Draw each halves of the line (left and right)
                Debug.DrawLine(line_starting_point, line_starting_point+(0.5f*neighbor_radius*parallel_vector), Color.cyan);
                Debug.DrawLine(line_starting_point, line_starting_point+(-0.5f*neighbor_radius*parallel_vector), Color.cyan);
            }
            
            Debug.Log("USING ORCA");
            
            //Solve quadratic programming problem according to orca_constraints and return new safe velocity
            Vector3 new_velocity = solveForNewVelocity();
            //Execute new safe velocity
            DriveAndRecover(new_velocity);
        }
        
        else //No constraints, all agents in the neighborhood give relative velocities that will not lead to collisions
        {
            DriveAndRecover(Vector3.zero); //follow path, recover if stuck
        }
    }

    private Vector3 solveForNewVelocity()
    {
        int numVars = 2; // v_x and v_z
        int numConstraints = orca_constraints.Count();

        desired_velocity = my_rigidbody.linearVelocity; //FOR NOW THE DESIRED VELOCITY IS SET AS THE CURRENT VELOCITY

        // Define matrix H (gives quadratic terms, found via calculations with pen and paper)
        double[,] H = { { 2, 0 }, { 0, 2 } };
        // Define vector c (gives linear terms, found via calculations with pen and paper)
        double[] c = { -2 * desired_velocity.x, -2 * desired_velocity.z };

        // Define ORCA constraints matrix A and RHS vector b, this is pre-allocation of the arrays
        double[,] A = new double[numConstraints, numVars];
        double[] b = new double[numConstraints]; //Acts as a lower bound, Av>=b
        double[] upper_bound = new double[numConstraints];

        for (int i = 0; i < numConstraints; i++)
        {
            var (p, n) = orca_constraints[i];
            A[i, 0] = n.x;
            A[i, 1] = n.z;
            b[i] = Vector3.Dot(p, n);
            upper_bound[i] = double.PositiveInfinity; //Sets the upper bound to +inf, meaning no upper bound
        }

        // Solve using ALGLIB's quadratic optimizer
        double[] solution;
        alglib.minqpreport report;
        alglib.minqpstate state;
        alglib.minqpcreate(numVars, out state);
        alglib.minqpsetquadraticterm(state, H, false); //false bool says to ALGLIB that H is symmetric, which it is.
        alglib.minqpsetlinearterm(state, c);
        alglib.minqpsetlc2dense(state, A, b, upper_bound); // Inequality constraints
        alglib.minqpsetbc(state, new double[] { -5, -5 }, new double[] { 5, 5 }); // Bounds for velocity to avoid impossibly high speeds
        alglib.minqpoptimize(state);
        alglib.minqpresults(state, out solution, out report);

        if (report.terminationtype > 3)
        {
            return new Vector3(0, 0, 0); // NO FEASIBLE SOLUTION FOUND return 0 vector
        }
        else
        {
            return new Vector3((float)solution[0], 0, (float)solution[1]); // Feasable solution found, return it.
        }
    }

    private void UpdateNeighboringAgents()
    {
        //Define local neighborhood where we look for other agents to perform ORCA on. A circle of some radius
        //Check the local neighborhood for other cars, return list of cars in neighborhood (sorted according to how close to this car they are?)

        List<(GameObject car, float distance)> sorted_cars = new List<(GameObject, float)>(); //This is a more modern way to use Tuples in C#
        
        //NOTE: for some reason, this car is added to m_OtherCars, so we have to exclude it below
        foreach (GameObject car in m_OtherCars)
        {
            if (car != gameObject && (transform.position - car.transform.position).magnitude < neighbor_radius) //Exclude this car from neighbor cars
            {
                sorted_cars.Add((car, (transform.position - car.transform.position).magnitude));
            }
        }
        
        //sort list of cars in ascending distance to this car, with lambda expression
        sorted_cars.Sort((tuple_a, tuple_b) => tuple_a.distance.CompareTo(tuple_b.distance));

        for (int i = 0; i < sorted_cars.Count; i++)
        {
            if (i < max_considered_neighbors - 1) neighbor_agents.Add(sorted_cars[i].car);
            else break; //If we only want to add 5 closest cars, let i go from 0 to 4 and then break out of for-loop, based on max_considered_neighbors
        }

        /* //This solution did not work as the ground plane, goal, and start positions also are placed in the Default layer
        LayerMask agentlayer = LayerMask.GetMask("Default"); //All agents are in the Defaul layer. Any other gameobject with a collider is in the Obstacle layer
        neighbor_agents.Clear(); //Clear list from old neighbor-agents before we add new ones below:
        Collider[] collider_hits = Physics.OverlapSphere(transform.position, neighbor_radius, agentlayer); //Returns array of colliders of gameobjects inside sphere

        foreach (Collider hit in collider_hits)
        {
            if (hit.gameObject != gameObject) neighbor_agents.Add(hit.gameObject); // Ignore collider of car using Overlapshere to find nearby agents' colliders
        }
        */
    }

    private void DriveAndRecover(Vector3 orca_velocity) 
    { //Follow waypoints and do collision recovery
        // Rotate the car's forward vector by �30� to get left/right directions for the raycast
        Vector3 directionRight = Quaternion.Euler(0, -30, 0) * transform.forward;
        Vector3 directionLeft = Quaternion.Euler(0, 30, 0) * transform.forward;
        Vector3 directionBackLeft = Quaternion.Euler(0, 150, 0) * transform.forward;
        Vector3 directionBackRight = Quaternion.Euler(0, -150, 0) * transform.forward;
        Vector3 directionBack = Quaternion.Euler(0, 180, 0) * transform.forward;

        RaycastHit hitRight; RaycastHit hitLeft; RaycastHit hitStraight; 
        RaycastHit hitBackRight; RaycastHit hitBackLeft; RaycastHit hitBack;
        float maxRangeClose = 7f;

        // Cast the ray in world space (6 ray cast in front,back and diagonals)
        bool obsRightClose = Physics.Raycast(transform.position, directionRight, out hitRight, maxRangeClose);
        bool obsLeftClose = Physics.Raycast(transform.position, directionLeft, out hitLeft, maxRangeClose);
        bool obsStraightClose = Physics.Raycast(transform.position, transform.TransformDirection(new Vector3(0, 0, 1)),
            out hitStraight, maxRangeClose);
        bool obsBackRightClose =
            Physics.Raycast(transform.position, directionBackRight, out hitBackRight, maxRangeClose);
        bool obsBackLeftClose = Physics.Raycast(transform.position, directionBackLeft, out hitBackLeft, maxRangeClose);
        bool obsBackClose = Physics.Raycast(transform.position, directionBack, out hitBack, maxRangeClose);

        // Draw Raycasts in Blue
        /* Debug.DrawRay(transform.position, directionRight * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionLeft * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionBackRight * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionBackLeft * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, directionBack * maxRangeClose, Color.blue);
        Debug.DrawRay(transform.position, transform.forward * maxRangeClose, Color.blue); */

        if (!isStuck)
        {
            Vector3 target_position; //Why is target_position local to this scope, but target_velocity is an attribute of the class?

            if (orca_velocity != Vector3.zero)
            {
                target_position = transform.position + orca_velocity;
                target_velocity = orca_velocity;
            }
            else
            {
                target_position = path_of_points[currentPathIndex]; 
                target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            }
            
            Debug.DrawLine(transform.position, transform.position + orca_velocity, Color.white, 1f); //Draws white line if we have nonzero orca velocity
            Debug.DrawLine(transform.position, path_of_points[currentPathIndex], Color.black); //Draws black line to waypoint 
            
            old_target_pos = target_position;

            float distance = Vector3.Distance(target_position, transform.position);

            // Scale k_p and k_d based on distance  between 1 and 10
            float scaleFactor = Mathf.Clamp(distance / 5f, 2f, 8f); // Adjust 5f to control sensitivity
            float k_p_dynamic = Mathf.Lerp(2f, 5f, scaleFactor / 5f);
            float k_d_dynamic = Mathf.Lerp(2f, 4f, scaleFactor / 4f);

            float k_v = Mathf.Lerp(1f, 2f, scaleFactor / 8f); // New gain factor for velocity feedback
            Vector3 velocity_damping = -k_v * my_rigidbody.linearVelocity;

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
            Vector3 desired_acceleration =
                k_p_dynamic * position_error + k_d_dynamic * velocity_error + velocity_damping;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            //Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            //Debug.DrawLine(transform.position, transform.position + my_rigidbody.linearVelocity, Color.blue);
            //Debug.DrawLine(transform.position, transform.position + desired_acceleration.normalized * 5, Color.yellow);

            // Turn if you're too close to an obstacle
            if (obsRightClose) steering += 10;
            if (obsLeftClose) steering -= 10;
            if (obsBackRightClose) steering += 10;
            if (obsBackLeftClose) steering -= 10;

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);

            float distToPoint = 6;
            if (currentPathIndex == path_of_points.Count() - 1)
            { distToPoint = 1; }

            if (Vector3.Distance(target_position, transform.position) < distToPoint)
            {
                checkNewPoint = true;
                currentPathIndex++;
            }

            // If you're barely moving it means you're stuck
            if (my_rigidbody.linearVelocity.magnitude < 0.5f && currentPathIndex > 1)
            {
                timeStuck += 1;
                if (timeStuck > 70) isStuck = true; // If you're not moving for too long you're stuck
            }
        }
        else //if stuck:
        {
            // If you have an obstacle behind you go forward
            if (obsBackClose || obsBackRightClose || obsBackLeftClose) m_Car.Move(0f, 100f, 100f, 0f);
            else m_Car.Move(0f, -100f, -100f, 0f); //go backwards

            timeStuck -= 1;
            if (timeStuck == 0) isStuck = false;
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

