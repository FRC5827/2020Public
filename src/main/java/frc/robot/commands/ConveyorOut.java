/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorOut extends CommandBase {
  /**
   * Creates a new ConveyerBeltSpeed.
   */
  
  private final ConveyorSubsystem m_conveyorBeltSub;
  private XboxController m_joystick;
  private double m_motorSpeed;

  public ConveyorOut(ConveyorSubsystem conveyorBeltSub, double speed, XboxController joy) {
    m_conveyorBeltSub = conveyorBeltSub;
    m_motorSpeed = speed;
    m_joystick = joy;
    addRequirements(m_conveyorBeltSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyorBeltSub.setSpeed(m_motorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyorBeltSub.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_joystick.getRawButton(2);
  }
}