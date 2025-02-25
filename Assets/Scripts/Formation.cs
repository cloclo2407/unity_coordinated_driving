using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Formation
{
    float minSameDirection = 25f; // how much too cars should have the same direction to start following each other
    float maxDeltaVelocity = 10f; // Maximum difference between the speed of two cars to start following each other

    // return a car to follow if one close going in the same direction
    // return null
    public GameObject LineFormation(CarController my_Car, GameObject[] m_OtherCars, Vector3 target_position)
    {
        GameObject carToFollow = null;
        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == my_Car) continue; // skip self

            Vector3 otherPosition = otherCar.transform.position;
            Vector3 otherVelocity = otherCar.GetComponent<Rigidbody>().linearVelocity;
            AIP1TrafficCar otherCarScript = otherCar.GetComponent<AIP1TrafficCar>(); // Get the script


            if (!otherCarScript.IsBeingFollowed && CanBeFollowed(my_Car.transform.position, my_Car.GetComponent<Rigidbody>().linearVelocity , otherPosition, otherVelocity, target_position))
            {
                carToFollow = otherCar;
                otherCarScript.IsBeingFollowed = true; // Set it to true after selecting
                break; // Exit loop after finding a car to follow
            }
        }
        return carToFollow;
    }

    // return true if the other car is in front of my car and their directions are similar and their velocity are similar
    // Suppressed velocity for now
    private bool CanBeFollowed (Vector3 myPosition, Vector3 myVelocity, Vector3 otherPosition, Vector3 otherVelocity, Vector3 target_position)
    {
        Vector3 deltaPosition = otherPosition - myPosition;
        float deltaVelocity = (otherVelocity - myVelocity).magnitude;
        float behind = Vector3.Dot(deltaPosition, otherVelocity);
        Vector3 my_direction = target_position - myPosition;
        float sameDirection = Vector3.Angle(my_direction.normalized, otherVelocity.normalized);
        if (behind > 0 && sameDirection < minSameDirection  && deltaPosition.magnitude < 10f)
        {
            return true;
        }
        return false;
    }

}