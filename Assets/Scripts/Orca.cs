using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Map;
using Scripts.Game;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Orca
{
    Rigidbody m_rigidbody;
    private GameObject m_Agent;
    private GameObject[] m_OtherCars;

    private float timeHorizon = 1f; //tau in the report, high tau -> low responsiveness, low tau -> high responsiveness?
    private List<Tuple<Vector3, Vector3>> orca_constraints = new List<Tuple<Vector3, Vector3>>();
    private int max_considered_neighbors = 5;
    private List<GameObject> neighbor_agents = new List<GameObject>();
    public float neighbor_radius = 16f;

    public Orca(GameObject m_Agent, Rigidbody m_rigidbody, GameObject[] m_OtherCars)
    {
        this.m_Agent = m_Agent;
        this.m_rigidbody = m_rigidbody;
        this.m_OtherCars = m_OtherCars;
    }

    public Vector3 EvadeCollisionWithORCA(String vehicle)
    {
        orca_constraints.Clear(); //Reset ORCA constraints
        Vector3 v_A = m_rigidbody.linearVelocity; //Let this be agent A and other be agent B
        Vector3 pos_A = m_Agent.transform.position;
        float car_radius = 0f;
        if (vehicle == "car") car_radius = 2.2f; //We approximate the car as a circular robot with radius 2.2 based on the prefab model
        else if (vehicle == "drone") car_radius = 0.5f; //The drone is approximated as a capsule with radius 0.5 in the prefab model 
        else Debug.Log("vehicle string parameter must be either car or drone!");
        
        foreach (GameObject other_agent in neighbor_agents)
        {
            //Check if we are on collision course with other agent. Compute and store constraint if that is the case.
            var rigidbody_B = other_agent.GetComponent<Rigidbody>();
            Vector3 v_B = rigidbody_B.linearVelocity;
            Vector3 pos_B = other_agent.transform.position;
            Vector3 relative_velocity = (v_A - v_B); //Is this the correct way of calculating relative_vel
            Vector3 relative_position = (pos_B - pos_A); // and relative_pos?

            float theta = Mathf.Asin((car_radius + car_radius) / relative_position.magnitude); //Angle defining if relative_vel is in velocity obstacle
            Vector3 max_considered_velocity = relative_velocity + relative_position / timeHorizon;

            //NOTE Mathf.Asin returns an angle in RADIANS, while Vector3.SignedAngle returns an angle in DEGREES
            float relative_vectors_signed_angle = Vector3.SignedAngle(relative_position, relative_velocity, Vector3.up) * Mathf.Deg2Rad;
            //angle <0 means relative_vel is rotated counterclockwise from relative_pos by angle degrees
            //angle >0 means relative_vel is rotated clockwise from relative_pos by angle degrees

            if (Mathf.Abs(relative_vectors_signed_angle) < theta && relative_velocity.magnitude < max_considered_velocity.magnitude)
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

        if (orca_constraints.Count > 0) //We have an optimization problem to solve
        {
            //Draw constraint-lines and corrsponding normals for debugging:
            foreach (Tuple<Vector3, Vector3> constraint in orca_constraints)
            {
                //Get vector parallel to line. constraint.Item1 is the point on the line, constraint.Item2 is the Vector3 normal vector to line
                Vector2 perpendicular2D = new Vector2(constraint.Item2.x, constraint.Item2.z);
                Vector2 parallel2D = Vector2.Perpendicular(perpendicular2D);
                Vector3 parallel_vector = new Vector3(parallel2D.x, 0, parallel2D.y);
                Vector3 line_starting_point = m_Agent.transform.position + constraint.Item1;

                //Draw normal to constraint-line
                //Debug.DrawLine(line_starting_point, line_starting_point + constraint.Item2 * 5f, Color.yellow);
                //Draw each halves of the line (left and right)
                //Debug.DrawLine(line_starting_point, line_starting_point + (0.5f * neighbor_radius * parallel_vector), Color.cyan);
                //Debug.DrawLine(line_starting_point, line_starting_point + (-0.5f * neighbor_radius * parallel_vector), Color.cyan);
            }

            //Solve quadratic programming problem according to orca_constraints and return new safe velocity
            Vector3 new_velocity = solveForNewVelocity();
            //Execute new safe velocity
            return new_velocity;
        }

        else //No constraints, all agents in the neighborhood give relative velocities that will not lead to collisions
        {
            return Vector3.zero; //follow path, recover if stuck
        }
    }

    private Vector3 solveForNewVelocity()
    {
        int numVars = 2; // v_x and v_z
        int numConstraints = orca_constraints.Count;

        // Got recommended to switch desired_velocity from current_velocity to the vector between the car's pos and its next waypoint:
        //Vector3 next_waypoint = path_of_points[currentPathIndex];
        //desired_velocity = transform.position - next_waypoint; //Current velocity worked better?
        Vector3 desired_velocity = m_rigidbody.linearVelocity; //FOR NOW THE DESIRED VELOCITY IS SET AS THE CURRENT VELOCITY

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

    public void UpdateNeighboringAgents()
    {   //Define local neighborhood where we look for other agents to perform ORCA on. A circle of some radius
        //Check the local neighborhood for other cars, return list of cars in neighborhood (sorted according to how close to this car they are?)
        neighbor_agents.Clear(); //Clear list from old neighbor-agents before we add new ones below:

        List<(GameObject car, float distance)> sorted_cars = new List<(GameObject, float)>(); //This is a more modern way to use Tuples in C#

        //NOTE: for some reason, this car is added to m_OtherCars, so we have to exclude it below
        foreach (GameObject car in m_OtherCars)
        {
            if (car != m_Agent && (m_Agent.transform.position - car.transform.position).magnitude < neighbor_radius) //Exclude this car from neighbor cars
            { sorted_cars.Add((car, (m_Agent.transform.position - car.transform.position).magnitude)); }
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

    public bool NeedOrca()
    {
        if (neighbor_agents.Count != 0) return true;
        else return false;
    }
}