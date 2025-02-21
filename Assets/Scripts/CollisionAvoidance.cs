using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;


public class CollisionAvoidance 
{
    private float maxTimeToCollision = 10000f; // Change the velocity only if the collision will happen sooner than in maxTimeToCollision
    private float safetyRadius = 100f; // minimum distance required between the centers of the two cars

    // Compute safe velocity to avoid collision
    ///////////////////
    // How to use in fixedUpdate:
    //  Call AvoidCollisions to adjust velocity
    //Vector3 safeVelocity = collisionAvoidance.AvoidCollisions(my_rigidbody.velocity, this, otherCars);

    // Compute adjusted steering for obstacle avoidance
    //Vector3 avoidanceSteering = (safeVelocity - my_rigidbody.velocity).normalized;
    //float obstacle_avoiding_steering = Vector3.Dot(avoidanceSteering, transform.right);

    // Blend the original steering with the avoidance steering
    //float final_steering = steering + 0.5f * obstacle_avoiding_steering; // Weighted blend

    // Apply control input to the car
    //m_Car.Move(final_steering, accel, accel, 0f);
    /////////////////
    public Vector3 AvoidCollisions(Vector3 myVelocity, CarController my_Car, GameObject[] m_OtherCars) 
    {
        float minTimeToCollision = maxTimeToCollision; // keep the collision that will happen first
        Vector3 bestAvoidanceVelocity = myVelocity; // velocity to avoid first collision

        foreach (var otherCar in m_OtherCars) // check for each car if there will be a collision
        {
            if (otherCar == my_Car) continue; // skip self

            Vector3 deltaPosition = otherCar.transform.position - my_Car.transform.position; 
            Vector3 deltaVelocity = otherCar.GetComponent<Rigidbody>().linearVelocity - my_Car.GetComponent<Rigidbody>().linearVelocity;
            
            // Check if the velocity is inside the velocity obstacle
            if (IsVelocityInsideVO(deltaPosition, deltaVelocity))
            {
                float timeToCollision = CalculateTimeToCollision(deltaPosition, deltaVelocity);
                if (timeToCollision >= 0 && timeToCollision < minTimeToCollision)
                {
                    minTimeToCollision = timeToCollision;
                    // Find a new velocity to avoid collision
                    bestAvoidanceVelocity = GetSafeVelocity(myVelocity, deltaPosition, deltaVelocity);
                }
            }
        }
      
        return bestAvoidanceVelocity;
        
    }

    // To determinate wether the car is going to hit another car with velocity obstacle
    private bool IsVelocityInsideVO(Vector3 deltaPosition, Vector3 deltaVelocity)
    {
        float angle = Vector3.Angle(deltaVelocity, deltaPosition);
        float maxAngle = Mathf.Atan(safetyRadius / deltaPosition.magnitude) * Mathf.Rad2Deg;
        return angle < maxAngle;
    }

    // Get a safe velocity vector outside of cone (right of the cone)
    private Vector3 GetSafeVelocity(Vector3 myVelocity, Vector3 deltaPosition, Vector3 deltaVelocity)
    {
        Vector3 rightDirection = new Vector3(-deltaPosition.z, 0, deltaPosition.x).normalized; // Right perpendicular to deltaPosition 
        Vector3 adjustedDirection = (myVelocity.normalized + rightDirection * 3f).normalized; 
        adjustedDirection = adjustedDirection * myVelocity.magnitude;
        return adjustedDirection;
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