using System;
using System.Collections.Generic;
using System.Linq;
using FormationGame;
using Scripts.Game;
using Scripts.Utils;
using UnityEngine;

public class GameManagerA2 : AbstractGameManager
{
    public float maxDistance = 5;
    private bool isComplete;
    private Dictionary<GameObject, Goal> vehicleToGoalMapping = new();
    private Dictionary<string, List<MultiVehicleGoal>> goalsByGroup;
    private Dictionary<string, List<GameObject>> vehiclesByGroup;

    //TODO: If free target count = free vehicle count, then 1 vehicle = 1 goal. Groups always cooperate.
    public override List<Goal> CreateGoals(List<GameObject> vehicles)
    {
        List<Goal> goals = new List<Goal>();
        var targets = mapManager.transform.FindAllChildrenWithTag("Target");

        vehiclesByGroup = vehicles.GroupBy(vehicle => vehicle.transform.parent.name)
            .ToDictionary(
                group => CreateKey(group.Key),
                group => group.ToList()
            );

        goalsByGroup = targets.GroupBy(target => target.transform.parent.name)
            .ToDictionary(
                group => CreateKey(group.Key),
                group => group.Select(target => CreateLoSGoal(target)).ToList()
            ).ToDictionary(
                pair => pair.Key,
                pair => pair.Value.Select(goal => new MultiVehicleGoal(goal, vehiclesByGroup.GetValueOrDefault(pair.Key))).ToList()
            );

        return goalsByGroup.Values
            .SelectMany(group => group.ToList())
            .Select(multi => (Goal)multi)
            .ToList();
    }

    public string CreateKey(string key)
    {
        if (key == "Targets") return "Free";
        return key;
    }

    private LineOfSightGoal CreateLoSGoal(GameObject target)
    {
        var goal = new LineOfSightGoal(mapManager.transform.TransformPoint(target.transform.position), maxDistance);

        var goalColorIndicator = target.GetComponent<GoalColorIndicator>();
        if (goalColorIndicator != null) goalColorIndicator.SetGoal(goal);

        return goal;
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

    public List<MultiVehicleGoal> GetGoals(GameObject vehicle)
    {
        var returnList = new List<MultiVehicleGoal>();

        var team = GetTeam(vehicle);
        var vehicleGoals = goalsByGroup[team];
        if (team != "Free") returnList.AddRange(goalsByGroup.GetValueOrDefault("Free", new List<MultiVehicleGoal>()));

        returnList.AddRange(vehicleGoals);
        return returnList;
    }

    public string GetTeam(GameObject vehicle)
    {
        return vehiclesByGroup.FirstOrDefault(x => x.Value.Contains(vehicle)).Key;
    }

    public List<GameObject> GetTeamVehicles(GameObject vehicle)
    {
        var team = GetTeam(vehicle);
        if (team == "Free") return new List<GameObject>() { vehicle };
        return vehiclesByGroup[team];
    }
}