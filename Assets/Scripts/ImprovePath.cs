using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;


public class ImprovePath
{
    public List<Vector3> SmoothSplineCatmullRom(List<Vector3> originalPath, int subdivisionsPerSegment = 5)
    {
        List<Vector3> smooth_path_of_points = new List<Vector3>();
        if (originalPath.Count < 2)
            return originalPath;
        // Add first waypoint
        smooth_path_of_points.Add(originalPath[0]);

        for (int i = 0; i < originalPath.Count - 1; i++)
        {
            // p0: previous point (or same as p1 if none)
            Vector3 p0 = (i == 0) ? originalPath[i] : originalPath[i - 1];
            // p1: current
            Vector3 p1 = originalPath[i];
            // p2: next
            Vector3 p2 = originalPath[i + 1];
            // p3: next-next or same as p2 if none
            Vector3 p3 = (i + 2 < originalPath.Count) ? originalPath[i + 2] : originalPath[i + 1];

            // Generate Catmull-Rom points
            for (int step = 1; step <= subdivisionsPerSegment; step++)
            {
                float t = (float)step / (float)subdivisionsPerSegment;
                Vector3 newPoint = CatmullRom(p0, p1, p2, p3, t);
                smooth_path_of_points.Add(newPoint);
            }
        }
        return smooth_path_of_points;
    }

    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Standard Catmull-Rom formula
        // More references: https://www.iquilezles.org/www/articles/minispline/minispline.htm
        Vector3 a = 2f * p1;
        Vector3 b = p2 - p0;
        Vector3 c = 2f * p0 - 5f * p1 + 4f * p2 - p3;
        Vector3 d = -p0 + 3f * p1 - 3f * p2 + p3;
        return 0.5f * (a + (b * t) + (c * t * t) + (d * t * t * t));
    }

    public List<Vector3> simplifyPath(List<Vector3> path, float epsilon) //use Ramer-Douglas-Peucker
    {
        if (path == null || path.Count < 3)
            return path;

        List<Vector3> simplifiedPath = new List<Vector3>();
        simplifiedPath.Add(path[0]);
        simplifyRecursive(path, 0, path.Count - 1, epsilon, simplifiedPath);
        simplifiedPath.Add(path[path.Count - 1]); // add last point

        return simplifiedPath;
    }

    private void simplifyRecursive(List<Vector3> path, int startIndex, int endIndex, float epsilon, List<Vector3> new_path)
    {
        if (endIndex <= startIndex + 1) // can't simplify two points
            return;

        float maxDistance = 0;
        int indexFurthest = 0;

        Vector3 startPoint = path[startIndex];
        Vector3 endPoint = path[endIndex];

        // Find the point farthest from the line segment
        for (int i = startIndex + 1; i < endIndex; i++)
        {
            float distance = perpendicularDistance(path[i], startPoint, endPoint);
            if (distance > maxDistance)
            {
                maxDistance = distance;
                indexFurthest = i;
            }
        }

        if (maxDistance > epsilon) // if points too far from line, keep it
        {
            simplifyRecursive(path, startIndex, indexFurthest, epsilon, new_path); // simplify first part of the segment
            new_path.Add(path[indexFurthest]); // add the point
            simplifyRecursive(path, indexFurthest, endIndex, epsilon, new_path); // simplify second part of the segment
        }
    }

    private float perpendicularDistance(Vector3 point, Vector3 lineStart, Vector3 lineEnd) // get distance from point to line
    {
        Vector3 line = lineEnd - lineStart;
        Vector3 projection = Vector3.Project(point - lineStart, line);
        Vector3 closestPoint = lineStart + projection;
        return Vector3.Distance(point, closestPoint);
    }
}