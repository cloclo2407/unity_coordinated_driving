using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Formation
{
    float minSameDirection = 35f; // how much too cars should have the same direction to start following each other (angle)
    float maxDeltaVelocity = 10f; // Maximum difference between the speed of two cars to start following each other
    float maxDistance = 20f;

    // return a car to follow if one close going in the same direction (returns closest one)
    // else return null
    public void LineFormation(CarController my_Car, GameObject[] m_OtherCars, Vector3 target_position)
    {
        AIP1TrafficCar myCarScript = my_Car.GetComponent<AIP1TrafficCar>(); // Get the script

        if (myCarScript.carToFollow != null)
        {
            Vector3 otherPosition = myCarScript.carToFollow.transform.position;
            Vector3 otherVelocity = myCarScript.carToFollow.GetComponent<Rigidbody>().linearVelocity;
           
            if (CanBeFollowed(my_Car.transform.position, my_Car.GetComponent<Rigidbody>().linearVelocity, otherPosition, otherVelocity, target_position))
            {
                return;
            }
        }


        GameObject closestCar = null;
        float closestDistance = float.MaxValue;

        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == my_Car) continue; // skip self

            Vector3 otherPosition = otherCar.transform.position;
            Vector3 otherVelocity = otherCar.GetComponent<Rigidbody>().linearVelocity;

            float distance = Vector3.Distance(my_Car.transform.position, otherPosition);


            if (distance < closestDistance && CanBeFollowed(my_Car.transform.position, my_Car.GetComponent<Rigidbody>().linearVelocity, otherPosition, otherVelocity, target_position))
            {
                closestCar = otherCar;
                closestDistance = distance;
            }
        }

        if (closestCar != null)
        { 
            myCarScript.carToFollow = closestCar.GetComponent<CarController>();
            AIP1TrafficCar otherCarScript = closestCar.GetComponent<AIP1TrafficCar>(); // Get the script

            if (!otherCarScript.IsBeingFollowed)
            {
                otherCarScript.IsBeingFollowed = true; // Set it to true after selecting
                otherCarScript.followingCar = my_Car;
                return; // Exit loop after finding a car to follow
            }
            else
            {
                otherCarScript.followingCar.GetComponent<AIP1TrafficCar>().carToFollow = my_Car; // the car following the car I want to follow is going to follow me
                otherCarScript.followingCar = my_Car;
                return;
            }
            
        }
        if (myCarScript.carToFollow != null)
        {
            myCarScript.carToFollow.GetComponent<AIP1TrafficCar>().IsBeingFollowed = false;
            myCarScript.carToFollow = null;
        }
        return;
    }

    // return true if the other car is in front of my car and their directions are similar and their velocity are similar
    // Suppressed velocity for now
    private bool CanBeFollowed (Vector3 myPosition, Vector3 myVelocity, Vector3 otherPosition, Vector3 otherVelocity, Vector3 target_position)
    {
        Vector3 deltaPosition = otherPosition - myPosition;
        float deltaVelocity = (otherVelocity - myVelocity).magnitude;
        float behind = Vector3.Dot(deltaPosition.normalized, otherVelocity.normalized);
        Vector3 my_direction = target_position - myPosition;
        float sameDirection = Vector3.Angle(my_direction.normalized, otherVelocity.normalized);
        if (behind > 0 && sameDirection < minSameDirection  && deltaPosition.magnitude < maxDistance)
        {
            return true;
        }
        return false;
    }

}