/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import static frc.robot.Constants.TurnPIDConstants;

/**
 * An example command that uses an example subsystem.
 */
public class TurnToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PIDController m_turnPIDController;
  private int m_counter;

  /**
   * Creates a new robotTurnDegrees.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param limelightSubsystem The LimelightSubsystem used by this command.
   */
  public TurnToTarget(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    m_turnPIDController = new PIDController(TurnPIDConstants.kpTurnRio, TurnPIDConstants.kiTurnRio, TurnPIDConstants.kdTurnRio);

    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_counter = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_limelightSubsystem.setLedMode(true);
    m_limelightSubsystem.setPipeline(1);

    m_turnPIDController.reset();
    m_turnPIDController.disableContinuousInput();
    m_turnPIDController.setTolerance(1.0);
    m_turnPIDController.setSetpoint(0.0);

    m_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = m_turnPIDController.calculate(m_limelightSubsystem.getTX());
    if (m_limelightSubsystem.hasValidTarget()) {
      m_driveSubsystem.tankDriveVelocity(-velocity, velocity);
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // leave LED on as we probably will use in subsequent command or change pipeline
    //m_limelightSubsystem.setLedMode(false);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_limelightSubsystem.hasValidTarget()) {
      m_counter++;
    }
    else {
      m_counter = 0;
    }

    // if we lost target for 10 consecutive scheduled executes, then stop
    if (m_counter >= 10) {
      return true;
    }
    return (m_turnPIDController.atSetpoint());
  }
}
