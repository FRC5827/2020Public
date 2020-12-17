/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.DriveConstants;


/**
 * An example command that uses an example subsystem.
 */
public class DriveToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;

  private Command m_command;

  public static TrajectoryConfig m_config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond_Low,
  DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(new DifferentialDriveVoltageConstraint(
                  new SimpleMotorFeedforward(DriveConstants.ksVolts_Low, DriveConstants.kvVoltSecondsPerMeter_Low,
                          DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                  DriveConstants.kDriveKinematics, 11))
          .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
          .setReversed(true);
  ;


  /**
   * Creates a new DriveToTarget.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param limelightSubsystem The LimelightSubsystem used by this command.
   */
  public DriveToTarget(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {

    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // m_command command MUST have the same dependency requirements
    addRequirements(driveSubsystem);
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // It is up to caller to ensure LEDs are on by enabling or selecting correct pipeline
    //m_limelightSubsystem.setLedMode(true);
    m_limelightSubsystem.setPipeline(1);

    m_command = getDriveTargetCommand(m_driveSubsystem, m_limelightSubsystem);
    m_command.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    // leave LED on as we probably will use in subsequent command
    //m_limelightSubsystem.setLedMode(false);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }

  // prior to calling, make sure we are pointing at the target with tx from Limelight
  // close to zero
  public Command getDriveTargetCommand(DriveSubsystem drive, LimelightSubsystem limelight) {
    double distanceToTarget;
    Pose2d currentPose;
    Pose2d startingPose;
    Pose2d endingPose;
    Translation2d midpoint;
    Trajectory trajectory;

    System.out.println("getDriveTargetCommand");

    // expects LEDs to have been turned on previously
    if (!limelight.hasValidTarget()) {
      // no target -- can't calculate trajectory or drive to target using limelight, so bail out
      System.out.println("No valid target");
      return new WaitCommand(0.0);
    }

    distanceToTarget = limelight.getDistance();

    //System.out.println("Distance to target: " + distanceToTarget);
    //System.out.println("Heading: " + drive.getPose().getRotation().getDegrees());
    //System.out.println("Calling DriveRamsete. Pose: " + drive.getPose().toString());

    currentPose = drive.getPose();

    try {
      // use current pose as starting point but flip rotation around as we are reversing through the trajectory points
      startingPose = new Pose2d(currentPose.getTranslation(), currentPose.getRotation().plus(Rotation2d.fromDegrees(0)));
      //System.out.println("startingPose: " + startingPose.toString());
      //System.out.println("distanceToTarget: " + distanceToTarget);
      endingPose = drive.calcEndPose(distanceToTarget);
      midpoint = drive.calcMidPoint(startingPose, endingPose);
      //System.out.println("midpoint: " + midpoint.toString());
      trajectory = TrajectoryGenerator.generateTrajectory(startingPose, List.of(midpoint), endingPose, m_config);
      // TODO -- if this works without issue, move to global constant
      return new DriveRamsete(trajectory, drive);//.withInterrupt(() -> (m_limelightSubsystem.getDistance() <= Units.metersToInches(1.6)));
    }
    catch (Exception e) {
      System.out.println("Caught exception with trajectory/Ramsete");
      System.out.println(e.toString());
      return new WaitCommand(0.0);
    }
  }
}
