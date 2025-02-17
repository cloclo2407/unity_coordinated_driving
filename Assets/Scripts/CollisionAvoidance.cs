using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;

public class CollisionAvoidance 
{
    private float maxTimeToCollision = 5f; // Change the velocity only if the collision will happen sooner than in maxTimeToCollision
    private float safetyRadius = 4.0f; // minimum distance required between the centers of the two cars


    private Vector3 AvoidCollisions(Vector3 myVelocity) 
    {
        foreach (var otherCar in m_OtherCars) // check for each car if there will be a collision
        {
            if (otherCar == gameObject) continue; // skip self

            Vector3 deltaPosition = otherCar.transform.position - transform.position; 
            Vector3 deltaVelocity = otherCar.GetComponent<Rigidbody>().velocity - m_Car.Rigidbody.velocity;
            
            // Check if the velocity is inside the velocity obstacle
            if (IsVelocityInsideVO(myVelocity, deltaPosition, deltaVelocity)
            {
                float timeToCollision = CalculateTimeToCollision(deltaPosition, deltaVelocity)
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
        Vector3 apex = deltaVelocity; // The tip of the cone
        Vector3 diff = velocity - apex;
        float distance = Vector3.Cross(diff, deltaPosition).magnitude / deltaPosition.magnitude; // Cross = perpendicular distance
        return distance < safetyRadius;
    }

    // Get a safe velocity vector outside of cone
    private Vector3 GetSafeVelocity(Vector3 myVelocity, Vector3 deltaPosition, Vector3 deltaVelocity)
    {
        Vector3 lateralDir = Vector3.Cross(deltaPosition, Vector3.up).normalized; // Perpendicular direction
        Vector3 newVelocity1 = deltaVelocity + lateralDir * safetyRadius; // left of the cone
        Vector3 newVelocity2 = deltaVelocity - lateralDir * safetyRadius; // right of the cone
        if (myVelocity - newVelocity1).sqrMagnitude < (myVelocity - newVelocity2).sqrMagnitude // keep the closest
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