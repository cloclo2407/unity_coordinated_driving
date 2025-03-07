using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Intersection
{
    float maxDistanceToStop = 6f;
    float minAngleToStop = 40f;

    public bool HasToStop(CarController myCar, GameObject[] m_OtherCars)
    {
        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == myCar) continue; // skip self

            if (otherCar.GetComponent<Rigidbody>().linearVelocity.magnitude < 0.1f) continue; // ignore if the car is not moving

            Vector3 otherPosition = otherCar.transform.position;
            AIP1TrafficCar otherCarScript = otherCar.GetComponent<AIP1TrafficCar>(); // Get the script
            Vector3 otherTarget = otherCarScript.target_position;

            Vector3 myPosition = myCar.transform.position;
            AIP1TrafficCar myCarScript = myCar.GetComponent<AIP1TrafficCar>(); // Get the script
            Vector3 myTarget = myCarScript.target_position;

            if (myCarScript.carToFollow != null)
            {
                if (myCarScript.carToFollow.GetComponent<AIP1TrafficCar>().hasToStop) return true; // Stop if the car I follow has to stop
                else return false; //only leading car decides
            }

            float angle = Vector3.Angle(myTarget - myPosition, otherTarget - otherPosition);

            // Check for intersection
            if (SegmentsIntersect(myPosition, myTarget, otherPosition, otherTarget, out Vector3 intersection))
            {
                // Check if the intersection is within maxDistanceToStop units from myPosition
                // and if you're not going into the same direction
                if (Vector3.Distance(myPosition, intersection) <= maxDistanceToStop && angle > minAngleToStop)
                {
                    if (otherCarScript.carToFollow == null && myCarScript.myCarIndex > otherCarScript.myCarIndex)
                        return true;
                    else if (otherCarScript.carToFollow != null) return true; // if the other car is following and I'm not, I stop
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