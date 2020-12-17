/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorServoCommand extends CommandBase {

  private final ColorWheelSubsystem m_subsystem;
  private Joystick m_joystick;
  private int button;
  private boolean m_up;

  public ColorServoCommand(ColorWheelSubsystem subsystem, Joystick joystick, int button, boolean input) {
    m_subsystem = subsystem;
    m_joystick = joystick;
    this.button = button;
    m_up = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_up) {
      m_subsystem.moveServoUp();
    } else {
      m_subsystem.moveServoDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_subsystem.moveServoDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_joystick.getRawButton(button);
  }
}