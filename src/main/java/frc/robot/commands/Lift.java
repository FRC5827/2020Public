/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Lift extends CommandBase {
  /**
   * Creates a new Lift.
   */
  private final ClimbSubsystem m_climb;
  private final Joystick m_joystick;
  private double m_speed;
  private int m_button;

  public Lift(ClimbSubsystem climb, Joystick joy, double speed, int button) {
    m_climb = climb;
    m_joystick = joy;
    m_speed = speed;
    m_button = button;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.moveClimb(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.moveClimb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_joystick.getRawButtonReleased(m_button);
  }
}
