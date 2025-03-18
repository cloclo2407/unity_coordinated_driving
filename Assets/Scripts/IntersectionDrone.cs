using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Vehicle;


public class IntersectionDrone
{
    private float minAngleToStop = 35f; // minimum angle between the orientation of two drones to stop (if their orientation is similar they don't stop because they can follow each other)
    private float translationDistance = 2f; // the distance to translate one segment of the path to left and right to check if the other path doesn't cross but is too close
    private int myFrontDifferent = 5;
    private int myFrontSimilar = 4;
    private int otherFront = 6;
    private int otherBackSimilar = 1;
    private int otherBackDifferent = 2;
    private float similarDirectionThreshold = 38f;
    /*
     * Function that returns a boolean to indicate if a drone has to stop because its path crosses another drone's path
     * 
     * If my path intersects with another drone that is not following: 
     * Stop if the angles between the two drones > minAngleToStop && the other drone doesn't have to stop && my index is bigger
     * 
     * If my path intersects with another drone that is following a drone:
     * Stop if the other drone doesn't have to stop && the angle between the two drones is > minAngleToStop
     */
    public bool HasToStop(DroneController myDrone, GameObject[] m_OtherDrones)
    {
        Vector3 myPosition = myDrone.transform.position;
        AIP2TrafficDrone myDroneScript = myDrone.GetComponent<AIP2TrafficDrone>(); // Get the script
        List<Vector3> myPath = myDroneScript.path_of_points;
        int myIndex = myDroneScript.currentPathIndex;
        Vector3 myWaypoint = myPath[myIndex];
        Vector3 myDirection = myWaypoint+Vector3.up*1.8f - myPosition; //The actual waypoints are below and in front of the drone

        foreach (var otherDrone in m_OtherDrones) // Check for all the other drones if my path is going to intersect their path
        {
            if (otherDrone == myDrone) continue; // skip self
            
            AIP2TrafficDrone otherDroneScript = otherDrone.GetComponent<AIP2TrafficDrone>();
            if (otherDroneScript.goal_reached) continue; // ignore drone who already reached their goal
            
            List<Vector3> otherPath = otherDroneScript.path_of_points;
            if (otherPath.Count() == 0 || myPath.Count() == 0) continue; // Ensure paths are valid
            
            Vector3 otherPosition = otherDrone.transform.position;
            int otherIndex = otherDroneScript.currentPathIndex;
            Vector3 otherWaypoint = otherPath[otherIndex];
            Vector3 otherDirection = otherWaypoint+Vector3.up*1.8f - otherPosition; //The actual waypoints are below and in front of the drone
            
            float angle = Vector3.Angle(myDirection, otherDirection); // calculate angle between the two drones' headings

            int myFront;
            int otherBack;
            if (angle > minAngleToStop)
            {
                myFront = myFrontDifferent;
                otherBack = otherBackDifferent;
            }
            else
            {
                myFront = myFrontSimilar;
                otherBack = otherBackSimilar;
            }

            for (int i = Mathf.Max(myIndex - 2, 0); i < Mathf.Min(myIndex + myFront, myPath.Count - 1); i++) // Check right behind me and the next few segments of my path
            {
                for (int j = Mathf.Max(otherIndex - otherBack, 0); j < Mathf.Min(otherIndex + otherFront, otherPath.Count - 1); j++) // Check right behind the other drone and the next few segments of its path
                {
                    Vector3 myStart = myPath[i];
                    Vector3 otherStart = otherPath[j];
                    Vector3 myEnd = myPath[i + 1];
                    Vector3 otherEnd = otherPath[j + 1];


                    if (SegmentsIntersect(myStart, myEnd, otherStart, otherEnd, out Vector3 intersection))
                    {
                        float myDistance = Vector3.Distance(myPosition, intersection);
                        float otherDistance = Vector3.Distance(otherPosition, intersection);

                        if (i >= myIndex && Mathf.Abs(myDistance - otherDistance) < 6f) // only stop if the intersection is not behind me
                        {
                            // Check if drones are moving in a similar direction
                            if (angle < similarDirectionThreshold)
                            {

                                Vector3 deltaPosition = otherPosition - myPosition;
                                float behind = Vector3.Dot(deltaPosition.normalized, myDirection.normalized);


                                if (behind > 0f && !otherDroneScript.hasToStop) // If my drone is behind
                                {
                                    return true;
                                }
                            }

                            else
                            {
                                if (myDroneScript.myDroneIndex > otherDroneScript.myDroneIndex && !otherDroneScript.hasToStop && myDistance > 5f)
                                {
                                    return true;
                                }
                            }

                        }
                    }
                    myStart = myEnd;
                    otherStart = otherEnd;
                }
            }
        }
        return false;
    }

    /*
     * Return true if the two segments are going to intersect, false otherwise
     * 
     * intersection contains the coordinates of the intersection if it exists
     */
    private bool SegmentsIntersect(Vector3 p1, Vector3 p2, Vector3 q1, Vector3 q2, out Vector3 intersection)
    {
        intersection = Vector3.zero;

        // Convert to 2D (ignore Y)
        Vector2 A = new Vector2(p1.x, p1.z);
        Vector2 B = new Vector2(p2.x, p2.z);
        Vector2 C = new Vector2(q1.x, q1.z);
        Vector2 D = new Vector2(q2.x, q2.z);

        // Define the normal vector of the line segment
        Vector2 AB = B - A;
        Vector2 CD = D - C;

        // Denominator for line intersection calculation
        float denominator = AB.x * (C.y - D.y) - AB.y * (C.x - D.x);

        // If denominator is 0, lines are parallel or coincident
        if (Mathf.Abs(denominator) < Mathf.Epsilon)
            return false;

        // Compute parameters for intersection point calculation
        float t = ((C.x - A.x) * (C.y - D.y) - (C.y - A.y) * (C.x - D.x)) / denominator;
        float u = ((C.x - A.x) * (A.y - B.y) - (C.y - A.y) * (A.x - B.x)) / denominator;

        // Check if intersection happens within the original segments
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            Vector2 intersection2D = A + t * AB;
            intersection = new Vector3(intersection2D.x, p1.y, intersection2D.y); // Assume Y remains the same
            return true;
        }

        // Now check if intersection occurs with translated segments to the right and left
        Vector2 translation = new Vector2(AB.y, -AB.x).normalized * translationDistance;

        // Translate both original segments to the left and right
        Vector2 A_right = A + translation;
        Vector2 B_right = B + translation;
        Vector2 C_right = C + translation;
        Vector2 D_right = D + translation;

        // Check for intersection with right-translated segment
        if (CheckSegmentIntersection(A_right, B_right, C, D, out intersection))
            return true;

        // Check for intersection with left-translated segment
        Vector2 A_left = A - translation;
        Vector2 B_left = B - translation;
        Vector2 C_left = C - translation;
        Vector2 D_left = D - translation;

        if (CheckSegmentIntersection(A_left, B_left, C, D, out intersection))
            return true;

        if (CheckSegmentIntersection(A_left, B_right, C, D, out intersection))
            return true;

        return false;
    }

    // Helper function to check intersection of two segments
    private bool CheckSegmentIntersection(Vector2 A, Vector2 B, Vector2 C, Vector2 D, out Vector3 intersection)
    {
        intersection = Vector3.zero;

        Vector2 AB = B - A;
        Vector2 CD = D - C;

        float denominator = AB.x * (C.y - D.y) - AB.y * (C.x - D.x);

        // If denominator is 0, lines are parallel or coincident
        if (Mathf.Abs(denominator) < Mathf.Epsilon)
            return false;

        float t = ((C.x - A.x) * (C.y - D.y) - (C.y - A.y) * (C.x - D.x)) / denominator;
        float u = ((C.x - A.x) * (A.y - B.y) - (C.y - A.y) * (A.x - B.x)) / denominator;

        // Check if intersection happens within both segments
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            Vector2 intersection2D = A + t * AB;
            intersection = new Vector3(intersection2D.x, 0f, intersection2D.y); // Assume Y remains the same
            return true;
        }

        return false;
    }


}