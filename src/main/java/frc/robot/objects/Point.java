package frc.robot.objects;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class Point {
    public Pose2d center, left, right;
    
    public Point(Pose2d center, Pose2d left, Pose2d right) {
        this.center = center;
        this.left = left;
        this.right = right;
    }
    
  /**
   * Returns the nearest Pose2d from a list of poses. If two or more poses in the list have the same
   * distance from this pose, return the one with the closest rotation component.
   *
   * @param poses The list of poses to find the nearest.
   * @return The nearest Pose2d from the list.
   */
  public static Point nearest(Pose2d pose, List<Point> points) {
    Point closestPoint = null;
    double closestDistance = Double.POSITIVE_INFINITY, closestAngle = Double.POSITIVE_INFINITY;
    for (Point point:points) {
        if (closestPoint == null) {
            closestPoint = point;
            closestDistance = point.center.getTranslation().getDistance(pose.getTranslation());
            closestAngle = Math.abs(point.center.getRotation().minus(pose.getRotation()).getRadians());
        } else {
            double newDistance = point.center.getTranslation().getDistance(pose.getTranslation());
            double newAngle = Math.abs(point.center.getRotation().minus(pose.getRotation()).getRadians());
            Boolean isDistanceCloser = newDistance < closestDistance;
            Boolean isDistanceEqual = newDistance == closestDistance;
            Boolean isAngleCloser = newAngle < closestAngle;
            if (isDistanceCloser || (isDistanceEqual && isAngleCloser)) {
                closestPoint = point;
                closestDistance = newDistance;
                closestAngle = newAngle;
            }
        }
    }
    return closestPoint;
  }
}
