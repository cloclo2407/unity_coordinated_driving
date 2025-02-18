using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;


public class CollisionAvoidance 
{
    private float maxTimeToCollision = 5f; // Change the velocity only if the collision will happen sooner than in maxTimeToCollision
    private float safetyRadius = 4.0f; // minimum distance required between the centers of the two cars

    private Vector3 AvoidCollisions(Vector3 myVelocity, CarController my_Car, GameObject[] m_OtherCars) 
    {
        foreach (var otherCar in m_OtherCars) // check for each car if there will be a collision
        {
            if (otherCar == my_Car) continue; // skip self

            Vector3 deltaPosition = otherCar.transform.position - my_Car.transform.position; 
            Vector3 deltaVelocity = otherCar.GetComponent<Rigidbody>().linearVelocity - my_Car.GetComponent<Rigidbody>().linearVelocity;
            
            // Check if the velocity is inside the velocity obstacle
            if (IsVelocityInsideVO(myVelocity, deltaPosition, deltaVelocity))
            {
                float timeToCollision = CalculateTimexToCollision(deltaPosition, deltaVelocity);
                if (timeToCollision >= 0 && timeToCollision < maxTimeToCollision)
                {
                    // Find a new velocity to avoid collision
                    myVelocity = GetSafeVelocity(myVelocity, deltaPosition, deltaVelocity);
                }
            }
        }

        return myVelocity;
    }

    // To determinate wether the car is going to hit another car with velocity obstacle
    private bool IsVelocityInsideVO(Vector3 velocity, Vector3 deltaPosition, Vector3 deltaVelocity)
    {
        Vector3 relativeVelocity = velocity - deltaVelocity;
        float angle = Vector3.Angle(relativeVelocity, deltaPosition);
        float maxAngle = Mathf.Atan(safetyRadius / deltaPosition.magnitude) * Mathf.Rad2Deg;
        return angle < maxAngle;
    }

    // Get a safe velocity vector outside of cone
    private Vector3 GetSafeVelocity(Vector3 myVelocity, Vector3 deltaPosition, Vector3 deltaVelocity)
    {
        Vector3 lateralDir = Vector3.Cross(deltaPosition, Vector3.up).normalized; // Perpendicular direction
        Vector3 newVelocity1 = deltaVelocity + lateralDir * safetyRadius; // left of the cone
        Vector3 newVelocity2 = deltaVelocity - lateralDir * safetyRadius; // right of the cone
        if ((myVelocity - newVelocity1).sqrMagnitude < (myVelocity - newVelocity2).sqrMagnitude) // keep the closest
        {
            return newVelocity1;
        }
        else
        {
            return newVelocity2;
        }
    }

    // Calculate the time to collision between two cars
    // Negative if no collision coming
    private float CalculateTimeToCollision(Vector3 deltaPosition, Vector3 deltaVelocity)
    {
        float relativeSpeedSquared = deltaVelocity.sqrMagnitude;
        if (relativeSpeedSquared == 0)
            return -1f;

        float t = -Vector3.Dot(deltaPosition, deltaVelocity) / relativeSpeedSquared;
        if (t > 0)
            return t;
        else
            return -1f;
    }

}