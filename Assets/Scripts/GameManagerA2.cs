using System.Collections.Generic;
using System.Linq;
using Scripts.Game;
using Scripts.Utils;
using UnityEngine;

public class GameManagerA2 : AbstractGameManager
{
    public float maxDistance = 5;
    private bool isComplete;
    public Dictionary<GameObject, Goal> vehicleToGoalMapping = new();

    public override List<Goal> CreateGoals(List<GameObject> vehicles)
    {
        List<Goal> goals = new List<Goal>();
        foreach (var mapManagerTargetPosition in mapManager.targetPositions)
        {
            var goal = new LineOfSightGoal(mapManager.transform.TransformPoint(mapManagerTargetPosition), maxDistance);
            goals.Add(goal);
            if (mapManager.startPositions.Count == mapManager.targetPositions.Count)
            {
                vehicleToGoalMapping[vehicles[goals.Count - 1]] = goal;
            }
        }

        var targets = mapManager.transform.Find("Grid/Targets");
        if (targets != null)
        {
            foreach (var indexedObject in targets.GetChildren().Select((value, index) => new { value, index }))
            {
                indexedObject.value.GetComponent<GoalColorIndicator>()?.SetGoal(goals[indexedObject.index]);
            }
        }


        return goals;
    }

    protected void FixedUpdate()
    {
        if (mapManager.startPositions.Count != mapManager.targetPositions.Count)
        {
            foreach (var vehicle in vehicleList)
            {
                foreach (var goal in goals)
                {
                    goal.CheckAchieved(vehicle);
                }
            }
        }
        else
        {
            vehicleToGoalMapping.ToList().ForEach(pair => pair.Value.CheckAchieved(pair.Key));
        }

        if (!isComplete)
        {
            completionTime = goals.Max(goals => goals.CurrentTime());
        }

        isComplete = goals.ToList().TrueForAll(goal => goal.IsAchieved());
    }
}