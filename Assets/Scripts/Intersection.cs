using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Intersection
{
    float minDistanceToStop = 10f; // Minimum stopping distance (slow speeds)
    float maxDistanceToStop = 15f; // Maximum stopping distance (high speeds)
    float maxSpeed = 23f; // Define max expected speed for scaling
    float minAngleToStop = 40f;
    private float translationDistance = 1f;

    public bool HasToStop(CarController myCar, GameObject[] m_OtherCars)
    {
        float mySpeed = myCar.GetComponent<Rigidbody>().linearVelocity.magnitude;
        float speedFactor = Mathf.Clamp01(mySpeed / maxSpeed); // Normalize speed to [0,1]
        float dynamicStopDistance = Mathf.Lerp(minDistanceToStop, maxDistanceToStop, speedFactor);

        Vector3 myPosition = myCar.transform.position;
        AIP1TrafficCar myCarScript = myCar.GetComponent<AIP1TrafficCar>(); // Get the script
        List<Vector3> myPath = myCarScript.path_of_points;
        int myIndex = myCarScript.currentPathIndex;

        if (myCarScript.carToFollow != null)
        {
            if (myCarScript.carToFollow.GetComponent<AIP1TrafficCar>().hasToStop)
            {
                return true; // Stop if the car I follow has to stop
            }
            
            else return false; //only leading car decides
        }

        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == myCar) continue; // skip self

            //if (otherCar.GetComponent<Rigidbody>().linearVelocity.magnitude < 0.1f) continue; // ignore if the car is not moving

            Vector3 otherPosition = otherCar.transform.position;
            AIP1TrafficCar otherCarScript = otherCar.GetComponent<AIP1TrafficCar>(); // Get the script
            List<Vector3> otherPath = otherCarScript.path_of_points;
            int otherIndex = otherCarScript.currentPathIndex;

            Vector3 myStart = myPosition;
            Vector3 otherStart = otherPosition;

            if (otherPath == null || myPath == null) continue; // Ensure paths are valid

            for (int i = Mathf.Max(myIndex-4, 0); i < Mathf.Min(myIndex + 5, myPath.Count - 1); i++)
            {
                for (int j = Mathf.Max(otherIndex-4, 0); j < Mathf.Min(otherIndex + 5, otherPath.Count - 1); j++)
                {
                    myStart = myPath[i];
                    otherStart = otherPath[j];
                    Vector3 myEnd = myPath[i+1];
                    Vector3 otherEnd = otherPath[j+1];

                    float angle = Vector3.Angle(myEnd - myStart, otherEnd - otherStart);
                    angle = Vector3.Angle(myCar.transform.forward, otherCar.transform.forward);

                    if (SegmentsIntersect(myStart, myEnd, otherStart, otherEnd, out Vector3 intersection))
                    {
                        if (i >= myIndex)
                        {
                                if (otherCarScript.carToFollow == null && angle > minAngleToStop && myCarScript.myCarIndex > otherCarScript.myCarIndex && !otherCarScript.hasToStop)
                                {
                                    return true;
                                }
                                else if (otherCarScript.carToFollow != null && !otherCarScript.carToFollow.GetComponent<AIP1TrafficCar>().hasToStop && angle > minAngleToStop) {
                                    return true;
                                }
                            //else if (!otherCarScript.hasToStop && myCarScript.hasToStop) return true;
                        }
                            if (Vector3.Distance(myPosition, intersection) <= Vector3.Distance(otherPosition, intersection) /*&& angle > minAngleToStop*/)
                        {

                            //if (otherCarScript.carToFollow == null && myCarScript.myCarIndex > otherCarScript.myCarIndex)
                            //    return true;
                            //else if (otherCarScript.carToFollow != null && !otherCarScript.hasToStop) return true;
                        }
                    }
                    myStart = myEnd;
                    otherStart = otherEnd;
                }
            }
        }
        return false;
    }

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