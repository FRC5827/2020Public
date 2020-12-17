/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.TurnPIDConstants;

public class LimelightDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PIDController m_turnPIDController;
  private final PIDController m_forwardPIDController;
  private int m_counter;

  /**
   * Creates a new LimelightDrive.
   */
  // TODO -- add as global constants
  public LimelightDriveCommand(DriveSubsystem drive, LimelightSubsystem limelight) {
    m_forwardPIDController = new PIDController(.10, 0.000, 0.010);
    m_turnPIDController = new PIDController(TurnPIDConstants.kpTurnRio, TurnPIDConstants.kiTurnRio, TurnPIDConstants.kdTurnRio);

    m_driveSubsystem = drive;
    m_limelightSubsystem = limelight;
    m_counter = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_limelightSubsystem.setLedMode(true);
    m_limelightSubsystem.setPipeline(1);

    m_forwardPIDController.reset();
    m_forwardPIDController.disableContinuousInput();
    m_forwardPIDController.setTolerance(1.0);
    m_forwardPIDController.setSetpoint(30.0);

    m_turnPIDController.reset();
    m_turnPIDController.disableContinuousInput();
    m_turnPIDController.setTolerance(1.0);
    m_turnPIDController.setSetpoint(0.0);

    m_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = m_forwardPIDController.calculate(m_limelightSubsystem.getDistance());
    double rotation = m_turnPIDController.calculate(m_limelightSubsystem.getTX());

    //output = 0.0;

    double outputLeft = output - rotation;
    double outputRight = output + rotation;

    if (m_limelightSubsystem.hasValidTarget()) {
      m_driveSubsystem.tankDriveVelocity(outputLeft, outputRight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // caller of command needs to disable LEDs
    //m_limelightSubsystem.setLedMode(false);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  // We check to see if there is a target, but it may not be visible because the 
  // led's were just enabled, so we keep a counter to check how many times the
  // target wasn't visible.
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
    return (m_forwardPIDController.atSetpoint());
  }
}
