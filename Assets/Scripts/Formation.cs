using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Formation
{
    float minSameDirection = 35f; // how much too cars should have the same direction to start following each other (angle)
    float maxDeltaVelocity = 5f; // Maximum difference between the speed of two cars to start following each other
    float maxDistance = 15f; // Maximum distance in a line
    float maxDistancePerpendicular = 6f; // Maximum distance in the direction perpendicular to the cars

    /*
     * Update attribute carToFollow of the AIP if there is a car to follow
     */
    public void LineFormation(CarController my_Car, GameObject[] m_OtherCars, Vector3 target_position)
    {
        AIP1TrafficCar myCarScript = my_Car.GetComponent<AIP1TrafficCar>(); // Get the script

        // if I'm already following a car
        if (myCarScript.carToFollow != null)
        {
            if (myCarScript.carToFollow.GetComponent<AIP1TrafficCar>().hasToStop) // if this car has to stop I continue following it
            {
                return;
            }
            
            // if this car doesn't have to stop I check if I can still follow it, if I can I continue and return 
            Vector3 otherPosition = myCarScript.carToFollow.transform.position;
            Vector3 otherVelocity = myCarScript.carToFollow.GetComponent<Rigidbody>().linearVelocity;
           
            if (CanBeFollowed(my_Car.transform.position, my_Car.GetComponent<Rigidbody>().linearVelocity, otherPosition, otherVelocity, target_position))
            {
                return;
            }
        }

        // I'm currently following no car
        GameObject closestCar = null; // contain the closest car I can follow
        float closestDistance = float.MaxValue; // distance to the closest car I can follow

        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == my_Car) continue; // skip self

            Vector3 otherPosition = otherCar.transform.position;
            Vector3 otherVelocity = otherCar.GetComponent<Rigidbody>().linearVelocity;
            float distance = Vector3.Distance(my_Car.transform.position, otherPosition);

            // If I can follow this car and it's the closest to me until now I update closestCar
            if (distance < closestDistance && CanBeFollowed(my_Car.transform.position, my_Car.GetComponent<Rigidbody>().linearVelocity, otherPosition, otherVelocity, target_position))
            {
                closestCar = otherCar;
                closestDistance = distance;
            }
        }

        // I've found a possible car to follow
        if (closestCar != null)
        { 
            myCarScript.carToFollow = closestCar.GetComponent<CarController>();
            AIP1TrafficCar otherCarScript = closestCar.GetComponent<AIP1TrafficCar>();

            // I check if this car is not already following me (can happen if they are next to each other)
            if (otherCarScript.carToFollow != null)
            {
                if (otherCarScript.carToFollow == my_Car)
                {
                    myCarScript.carToFollow = null ; //don't follow if it's following you
                    return;
                }
            }

            // If the car I want to follow is not followed by any car
            if (!otherCarScript.IsBeingFollowed)
            {
                otherCarScript.IsBeingFollowed = true; // Set it to true after selecting
                otherCarScript.followingCar = my_Car;
                return; // Exit loop after finding a car to follow
            }

            //If the car I want to follow is already followed by another car
            else
            {
                // Insertion into the formation
                otherCarScript.followingCar.GetComponent<AIP1TrafficCar>().carToFollow = my_Car; // the car following the car I want to follow is going to follow me
                otherCarScript.followingCar = my_Car;
                return;
            }
            
        }

        // If I didn't find a car to follow I update that I'm not following anymore
        if (myCarScript.carToFollow != null)
        {
            myCarScript.carToFollow.GetComponent<AIP1TrafficCar>().IsBeingFollowed = false;
            myCarScript.carToFollow = null;
        }
        return;
    }

    /*
     * Return true if I can follow the other car, false otherwise
     * 
     * You can follow the car if you are behind it
     * && the angle between your directions is < minSameDirection
     * && you're not too far on the side
     * && you're not too far behind
     * && your velocities are close
     */
    private bool CanBeFollowed (Vector3 myPosition, Vector3 myVelocity, Vector3 otherPosition, Vector3 otherVelocity, Vector3 target_position)
    {
        if (otherVelocity.magnitude <= 0) return false; // Don't follow a car that is not moving

        Vector3 deltaPosition = otherPosition - myPosition;
        float deltaVelocity = (otherVelocity - myVelocity).magnitude;
        float behind = Vector3.Dot(deltaPosition.normalized, otherVelocity.normalized);
        Vector3 my_direction = target_position - myPosition;
        float sameDirection = Vector3.Angle(my_direction.normalized, otherVelocity.normalized);

        float distanceParallel = Vector3.Dot(deltaPosition, otherVelocity.normalized); // Distance to the side of the car
        float distancePerpendicular = (deltaPosition - (distanceParallel * otherVelocity.normalized)).magnitude; // Distance behind the car

        if (behind > 0 && sameDirection < minSameDirection  && distanceParallel < maxDistance && distancePerpendicular < maxDistancePerpendicular  && deltaVelocity < maxDeltaVelocity)
        {
            return true;
        }
        return false;
    }

}