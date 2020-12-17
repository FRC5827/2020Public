package frc.robot.resources;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoDefinitions;
import frc.robot.Constants.DriveConstants;

public class TrajectoryModifier {

        private static int arrayLength = 4;
        private static Trajectory[] autoListTrajectories = new Trajectory[arrayLength];
        private static Trajectory[] autoListReverse0Trajectories = new Trajectory[arrayLength];
        private static Trajectory[] autoListReverse1Trajectories = new Trajectory[arrayLength];
        private static Trajectory[] autoListSlowTrajectories = new Trajectory[arrayLength];

        private static AutonomousHolder[] autoList = {
                        new AutonomousHolder(AutoDefinitions.autoLineEndPoints[0], AutoDefinitions.autoLineEndPoints[1],
                                        AutoDefinitions.autoLinePoints, false),
                        new AutonomousHolder(AutoDefinitions.trenchRunEndPoints[0],
                                        AutoDefinitions.trenchRunEndPoints[1], AutoDefinitions.trenchRunPoints, false),
                        new AutonomousHolder(AutoDefinitions.dumpEndPoints[0], AutoDefinitions.dumpEndPoints[1],
                                        AutoDefinitions.dumpPoints, false),
                        new AutonomousHolder(AutoDefinitions.stealEndPoints[0], AutoDefinitions.stealEndPoints[1],
                                        AutoDefinitions.stealPoints) };

        private static AutonomousHolder[] autoListReverse0 = { new AutonomousHolder(false),
                        new AutonomousHolder(AutoDefinitions.trenchRunEndPointsReverse0[0],
                                        AutoDefinitions.trenchRunEndPointsReverse0[1],
                                        AutoDefinitions.trenchRunPointsReverse0, true),
                        new AutonomousHolder(AutoDefinitions.dumpEndPointsReverse0[0],
                                        AutoDefinitions.dumpEndPointsReverse0[1], AutoDefinitions.dumpPointsReverse0,
                                        true),
                        new AutonomousHolder(AutoDefinitions.stealEndPointsReverse[0], AutoDefinitions.stealEndPointsReverse[1], AutoDefinitions.stealPoints, true) };

        private static AutonomousHolder[] autoListReverse1 = { new AutonomousHolder(false),
                        new AutonomousHolder(AutoDefinitions.trenchRunEndPointsReverse[0],
                                        AutoDefinitions.trenchRunEndPointsReverse[1],
                                        AutoDefinitions.trenchRunPointsReverse, true),
                        new AutonomousHolder(false),
                        new AutonomousHolder(false) };

        private static AutonomousHolder[] autoListSlow = { new AutonomousHolder(false),
                        new AutonomousHolder(AutoDefinitions.trenchRunEndPointsSlow[0],
                                        AutoDefinitions.trenchRunEndPointsSlow[1], AutoDefinitions.trenchRunPointsSlow,
                                        true),
                        new AutonomousHolder(false),
                        new AutonomousHolder(false)};

        public static AutonomousHolder[] getAutoList() {
                return autoList;
        }

        public static AutonomousHolder[] getAutoListReverse0() {
                return autoListReverse0;
        }

        public static AutonomousHolder[] getAutoListReverse1() {
                return autoListReverse1;
        }

        public static AutonomousHolder[] getAutoListSlow() {
                return autoListSlow;
        }

        public static TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond_High,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared_High)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        //.addConstraint(new DifferentialDriveVoltageConstraint(
                                        //                new SimpleMotorFeedforward(DriveConstants.ksVolts_High,
                                        //                                DriveConstants.kvVoltSecondsPerMeter_High,
                                        //                                DriveConstants.kaVoltSecondsSquaredPerMeter_High),
                                        //                DriveConstants.kDriveKinematics, 11))
                                        .addConstraint(new CentripetalAccelerationConstraint(
                                                        DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq));

        public static TrajectoryConfig slowConfig = new TrajectoryConfig(
                        DriveConstants.kMaxSpeedMetersPerSecond_Low / 2,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low / 2)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        .addConstraint(new DifferentialDriveVoltageConstraint(
                                                        new SimpleMotorFeedforward(DriveConstants.ksVolts_Low,
                                                                        DriveConstants.kvVoltSecondsPerMeter_Low,
                                                                        DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                                                        DriveConstants.kDriveKinematics, 10))
                                        .addConstraint(new CentripetalAccelerationConstraint(
                                                        DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(false);

        public static TrajectoryConfig reverseConfig = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond_High,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared_High)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        //.addConstraint(new DifferentialDriveVoltageConstraint(
                                        //                new SimpleMotorFeedforward(DriveConstants.ksVolts_High,
                                        //                                DriveConstants.kvVoltSecondsPerMeter_High,
                                        //                                DriveConstants.kaVoltSecondsSquaredPerMeter_High),
                                        //                DriveConstants.kDriveKinematics, 11))
                                        .addConstraint(new CentripetalAccelerationConstraint(
                                                        DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(true);

        // Get the information from the sendable chooser used in Robot.java and use the
        // correct autonomous
        public static Trajectory chooseTrajectory(SendableChooser<Integer> chooser) {
                // Get sendable chooser here.
                //AutonomousHolder auton = autoList[chooser.getSelected()];
                //return genTrajectory(auton.getStartPoint(), auton.getEndPoint(), auton.getWaypoints());
                return autoListTrajectories[chooser.getSelected()];
        }

        public static Trajectory chooseReverseTrajectory1(SendableChooser<Integer> chooser) {
                //AutonomousHolder auton = autoListReverse1[chooser.getSelected()];
                /*
                 * if (!auton.isReversed()) { return genReverseTrajectory(new Pose2d(0, 0, new
                 * Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), List.of()); }
                 */
                //return genReverseTrajectory(auton.getStartPoint(), auton.getEndPoint(), auton.getWaypoints());
                return autoListReverse1Trajectories[chooser.getSelected()];
        }

        public static Trajectory chooseReverseTrajectory0(SendableChooser<Integer> chooser) {
                //AutonomousHolder auton = autoListReverse0[chooser.getSelected()];
                /*
                 * if (!auton.isReversed()) { return genReverseTrajectory(new Pose2d(0, 0, new
                 * Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), List.of()); }
                 */
                //return genReverseTrajectory(auton.getStartPoint(), auton.getEndPoint(), auton.getWaypoints());
                return autoListReverse0Trajectories[chooser.getSelected()];
        }

        public static Trajectory chooseSlowTrajectory(SendableChooser<Integer> chooser) {
                //AutonomousHolder auton = autoListSlow[chooser.getSelected()];
                /*
                 * if (!auton.isReversed()) { return genReverseTrajectory(new Pose2d(0, 0, new
                 * Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), List.of()); }
                 */
                //return genSlowTrajectory(auton.getStartPoint(), auton.getEndPoint(), auton.getWaypoints());
                return autoListSlowTrajectories[chooser.getSelected()];
        }

        // generates a trajectory based upon the inputed end points and the interior
        // waypoints
        public static Trajectory genTrajectory(Pose2d start, Pose2d end, List<Translation2d> points) {
                return (TrajectoryGenerator.generateTrajectory(start, points, end, config));
        }

        public static Trajectory genSlowTrajectory(Pose2d start, Pose2d end, List<Translation2d> points) {
                return (TrajectoryGenerator.generateTrajectory(start, points, end, slowConfig));
        }

        public static Trajectory genReverseTrajectory(Pose2d start, Pose2d end, List<Translation2d> points) {
                return (TrajectoryGenerator.generateTrajectory(start, points, end, reverseConfig));
        }

        public static Trajectory genQuinticTrajectory(List<Pose2d> points) {
                return (TrajectoryGenerator.generateTrajectory(points, config));
        }

        public static void genAllTrajectories() {
                for (int i = 0; i < arrayLength; i++) {
                        if (autoList[i].getStartPoint() != null) {
                                autoListTrajectories[i] = genTrajectory(autoList[i].getStartPoint(), autoList[i].getEndPoint(), autoList[i].getWaypoints());
                        }
                        else {
                                autoListTrajectories[i] = null;
                        }

                        if (autoListReverse1[i].getStartPoint() != null) {
                                autoListReverse1Trajectories[i] = genReverseTrajectory(autoListReverse1[i].getStartPoint(), autoListReverse1[i].getEndPoint(), autoListReverse1[i].getWaypoints());
                        }
                        else {
                                autoListReverse1Trajectories[i] = null;
                        }

                        if (autoListReverse0[i].getStartPoint() != null) {
                                autoListReverse0Trajectories[i] = genReverseTrajectory(autoListReverse0[i].getStartPoint(), autoListReverse0[i].getEndPoint(), autoListReverse0[i].getWaypoints());
                        }
                        else {
                                autoListReverse0Trajectories[i] = null;
                        }

                        if (autoListSlow[i].getStartPoint() != null) {
                                autoListSlowTrajectories[i] = genSlowTrajectory(autoListSlow[i].getStartPoint(), autoListSlow[i].getEndPoint(), autoListSlow[i].getWaypoints());
                        }
                        else {
                                autoListSlowTrajectories[i] = null;
                        }
                }
                return;
        }
}