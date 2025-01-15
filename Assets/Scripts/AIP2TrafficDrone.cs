using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Vehicle;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : MonoBehaviour
{
    private DroneController m_Drone;


    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        // See AIP1TrafficCar for how to get info from the world.
    }


    private void FixedUpdate()
    {
        // Sinusoidal example movement of Drone
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);
    }
}