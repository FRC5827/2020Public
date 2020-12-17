/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class DumpBalls extends CommandBase {
  private ConveyorSubsystem m_conveyor;
  private double m_time;
  private Timer timer = new Timer();
  /**
   * Creates a new DumpBalls.
   */
  public DumpBalls(ConveyorSubsystem conveyor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = conveyor;
    m_time = time;
    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.  
  @Override
  public void initialize() {
    System.out.println("init for dump");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.setSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.setSpeed(0.0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(m_time);
  }
}
