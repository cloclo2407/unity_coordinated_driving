using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;

public class Formation
{
    float minSameDirection = 0.90f; // how much too cars should have the same direction to start following each other
    float maxDeltaVelocity = 10f; // Maximum difference between the speed of two cars to start following each other

    // return a car to follow if one close going in the same direction
    // return null
    CarController Vector3 LineFormation(Vector3 myPosition, Vector3 myVelocity)
    {
        CarController carToFollow = null;
        for (var otherCar in m_OtherCars)
        {
            if (otherCar == gameObject) continue; // skip self

            Vector3 otherPosition = otherCar.Position;
            Vector3 otherVelocity = otherCar.Velocity;

            if (CanBeFollowed(myPosition, myVelocity, otherPosition, otherVelocity))
            {
                carToFollow = otherCar;
            }
        }
        return carToFollow;
    }

    // return true if the other car is in front of my car and their directions are similar and their velocity are similar
    private bool CanBeFollowed (Vector3 myPosition, Vector3 myVelocity, Vector3 otherPosition, Vector3 otherVelocity)
    {
        float deltaPosition = motherPosition - myPosition;
        float deltaVelocity = Vector3.Norm(otherVelocity - myVelocity);
        float behind = VectorDot(deltaPosition, otherVelocity);
        float sameDirection = Vector3.Dot(myVelocity.normalized, otherVelocity.normalized);
        if (behind > 0 && sameDirection > minSameDirection && deltaVelocity < maxDeltaVelocity)
        {
            return true;
        }
        return false;
    }

}