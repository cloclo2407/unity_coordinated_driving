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
    float minAngleToStop = 46f;

    public bool HasToStop(GameObject myAgent, GameObject[] m_OtherCars)
    {
        float mySpeed = myAgent.GetComponent<Rigidbody>().linearVelocity.magnitude;
        float speedFactor = Mathf.Clamp01(mySpeed / maxSpeed); // Normalize speed to [0,1]
        float dynamicStopDistance = Mathf.Lerp(minDistanceToStop, maxDistanceToStop, speedFactor);

        Vector3 myPosition = myAgent.transform.position;
        AIP1TrafficCar myCarScript = myAgent.GetComponent<AIP1TrafficCar>(); // Get the script
        List<Vector3> myPath = myCarScript.path_of_points;
        int myIndex = myCarScript.currentPathIndex;

        if (myCarScript.carToFollow != null)
        {
            if (myCarScript.carToFollow.GetComponent<AIP1TrafficCar>().hasToStop) return true; // Stop if the car I follow has to stop
            else return false; //only leading car decides
        }

        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == myAgent) continue; // skip self

            //if (otherCar.GetComponent<Rigidbody>().linearVelocity.magnitude < 0.1f) continue; // ignore if the car is not moving

            Vector3 otherPosition = otherCar.transform.position;
            AIP1TrafficCar otherCarScript = otherCar.GetComponent<AIP1TrafficCar>(); // Get the script
            List<Vector3> otherPath = otherCarScript.path_of_points;
            int otherIndex = otherCarScript.currentPathIndex;

            Vector3 myStart = myPosition;
            Vector3 otherStart = otherPosition;

            if (otherPath == null || myPath == null) continue; // Ensure paths are valid

            for (int i = Mathf.Max(myIndex-2, 0); i < Mathf.Min(myIndex + 5, myPath.Count - 1); i++)
            {
                for (int j = Mathf.Max(otherIndex-2, 0); j < Mathf.Min(otherIndex + 5, otherPath.Count - 1); j++)
                {
                    myStart = myPath[i];
                    otherStart = otherPath[j];
                    Vector3 myEnd = myPath[i+1];
                    Vector3 otherEnd = otherPath[j+1];

                    float angle = Vector3.Angle(myEnd - myStart, otherEnd - otherStart);
                    angle = Vector3.Angle(myAgent.transform.forward, otherCar.transform.forward);

                    if (SegmentsIntersect(myStart, myEnd, otherStart, otherEnd, out Vector3 intersection))
                    {
                        if (i >= myIndex)
                        {
                            if (otherCarScript.carToFollow == null && angle > minAngleToStop && myCarScript.myCarIndex > otherCarScript.myCarIndex && !otherCarScript.hasToStop /*&& Vector3.Distance(myPosition, intersection) < Vector3.Distance(otherPosition, intersection)*/)
                                return true;
                            else if (otherCarScript.carToFollow != null && !otherCarScript.hasToStop) return true;
                            else if (!otherCarScript.hasToStop && myCarScript.hasToStop) return true;
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

    private bool SegmentsIntersect(Vector3 p1, Vector3 p2, Vector3 q1, Vector3 q2, out Vector3 intersection) // return the intersection in intersection
    {
        intersection = Vector3.zero;

        // Convert to 2D (ignore Y)
        Vector2 A = new Vector2(p1.x, p1.z);
        Vector2 B = new Vector2(p2.x, p2.z);
        Vector2 C = new Vector2(q1.x, q1.z);
        Vector2 D = new Vector2(q2.x, q2.z);

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
            intersection = new Vector3(intersection2D.x, p1.y, intersection2D.y); // Assume Y remains the same
            return true;
        }

        return false;
    }

}