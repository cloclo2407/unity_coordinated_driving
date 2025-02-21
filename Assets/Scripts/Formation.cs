using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;

public class Formation
{
    float minSameDirection = 0.90f; // how much too cars should have the same direction to start following each other
    float maxDeltaVelocity = 10f; // Maximum difference between the speed of two cars to start following each other

    // return a car to follow if one close going in the same direction
    // return null
    private GameObject LineFormation(CarController my_Car, GameObject[] m_OtherCars)
    {
        GameObject carToFollow = null;
        foreach (var otherCar in m_OtherCars)
        {
            if (otherCar == my_Car) continue; // skip self

            Vector3 otherPosition = otherCar.transform.position;
            Vector3 otherVelocity = otherCar.GetComponent<Rigidbody>().linearVelocity;

            if (CanBeFollowed(my_Car.transform.position, my_Car.GetComponent<Rigidbody>().linearVelocity , otherPosition, otherVelocity))
            {
                carToFollow = otherCar;
            }
        }
        return carToFollow;
    }

    // return true if the other car is in front of my car and their directions are similar and their velocity are similar
    private bool CanBeFollowed (Vector3 myPosition, Vector3 myVelocity, Vector3 otherPosition, Vector3 otherVelocity)
    {
        Vector3 deltaPosition = otherPosition - myPosition;
        float deltaVelocity = (otherVelocity - myVelocity).magnitude;
        float behind = Vector3.Dot(deltaPosition, otherVelocity);
        float sameDirection = Vector3.Dot(myVelocity.normalized, otherVelocity.normalized);
        if (behind > 0 && sameDirection > minSameDirection && deltaVelocity < maxDeltaVelocity)
        {
            return true;
        }
        return false;
    }

}