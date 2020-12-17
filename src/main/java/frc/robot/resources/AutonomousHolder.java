/*
 *  Class meant to hold information for start point, interior waypoints, end points
 * 
 * @author Anirudh
*/

package frc.robot.resources;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class AutonomousHolder {
    
    private Pose2d start;
    private Pose2d end;
    private List<Translation2d> waypoints;
    private List<Pose2d> quinticWaypoints;
    private boolean reversed = false;

    /**
     * 
     * @param startPoint 
     * @param endPoint
     * @param points
     */
    public AutonomousHolder(Pose2d startPoint, Pose2d endPoint, List<Translation2d> points) {
        start = startPoint;
        end = endPoint;
        waypoints = points;
    }

    public AutonomousHolder(Pose2d startPoint, Pose2d endPoint, List<Translation2d> points, boolean reverse) {
        start = startPoint;
        end = endPoint;
        waypoints = points;
        reversed = reverse;
    }

    public AutonomousHolder(boolean reverse) {
        reversed = reverse;
    }

    /*public AutonomousHolder(List<Pose2d> points) {
        quinticWaypoints = points;
    }*/

    public Pose2d getStartPoint() {
        return start;
    }

    public Pose2d getEndPoint() {
        return end;
    }

    public List<Translation2d> getWaypoints() {
        return waypoints;
    }

    public List<Pose2d> getQuinticWaypoints() {
        return quinticWaypoints;
    }

    public boolean isReversed() {
        return reversed;
    }
}