/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSubsystem;

public class ControlPanelManual extends CommandBase {
  /**
   * Creates a new ControlPanelManual.
   */

  private final ColorWheelSubsystem m_color;
  private final Joystick m_joystick;
  private int button;
  private boolean right;

  public ControlPanelManual(ColorWheelSubsystem color, Joystick joy, int button, boolean right) {
    m_color = color;
    m_joystick = joy;
    this.right = right;
    this.button = button;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(right) {
      m_color.turnWheelClock();
    } else {
      m_color.turnWheelAnti();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_color.turnMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_joystick.getRawButton(button);
  }
}
