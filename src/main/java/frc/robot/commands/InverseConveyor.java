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
import frc.robot.subsystems.ConveyorSubsystem;

public class InverseConveyor extends CommandBase {
  /**
   * Creates a new InverseConveyor.
   */
  private final ConveyorSubsystem m_conveyor;
  private final Joystick m_buttonBoard;
  private double m_speed;

  public InverseConveyor(ConveyorSubsystem convey, double speed, Joystick buttonBoard) {
    m_conveyor = convey;
    m_buttonBoard = buttonBoard;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_buttonBoard.getRawButtonReleased(Constants.ButtonBoardIDs.kManualConveyorOut);
  }
}
