using System.Collections.Generic;
using Scripts.Game;
using UnityEngine;

namespace FormationGame
{
    public class MultiVehicleGoal : Goal
    {
        private readonly Goal trackedGoal;
        private readonly List<GameObject> teamVehicles;

        public MultiVehicleGoal(Goal underlying, List<GameObject> objects)
        {
            trackedGoal = underlying;
            teamVehicles = objects;
        }

        public bool CheckAchieved(GameObject objectToCheck)
        {
            if (teamVehicles.Contains(objectToCheck))
            {
                return CheckAchieved(objectToCheck);
            }

            return false;
        }

        public bool IsAchieved()
        {
            return trackedGoal.IsAchieved();
        }

        public void RestartTimer()
        {
            trackedGoal.RestartTimer();
        }

        public float CurrentTime()
        {
           return trackedGoal.CurrentTime();
        }
    }
}