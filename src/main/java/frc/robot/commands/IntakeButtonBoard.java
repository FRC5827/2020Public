/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeButtonBoard extends CommandBase {
  /**
   * Creates a new IntakeButtonBoard.
   */
  private final IntakeSubsystem m_intake;
  private final Joystick m_joystick;
  private double m_power;

  public IntakeButtonBoard(IntakeSubsystem intake, Joystick joy, double power) {
    m_intake = intake;
    m_joystick = joy;
    m_power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intake(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.intake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_joystick.getRawButtonReleased(Constants.ButtonBoardIDs.kIntakeDirectionOut);
  }
}
