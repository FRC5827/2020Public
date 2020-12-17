/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeIndicator;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ClimbMove extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private ClimbSubsystem m_climb;
  private PneumaticSubsystem m_pneumatics;
  private IntakeSubsystem m_intake;
  private Joystick m_buttonBoard;
  private double speed;
  private XboxController joy;
  private Timer timer = new Timer();
  private boolean enabled;

  /**
   * Creates a new ClimbMove.
   *
   * @param climb The climb used by this command.
   */
  public ClimbMove(ClimbSubsystem climb, PneumaticSubsystem pneumatics, IntakeSubsystem intake, XboxController joy) {
    m_climb = climb;
    m_pneumatics = pneumatics;
    m_intake = intake;
    this.joy = joy;
    // Use addRequirements() here to declare climb dependencies.
    addRequirements(climb);
    addRequirements(intake);
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (joy.getRawAxis(5) > 0.04) {
      m_climb.setCoastMode();
    } else {
      m_climb.setBrakeMode();
    }
    m_intake.setIntake(IntakeIndicator.up);

    timer.start();
    m_pneumatics.unlockWinch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_climb.moveWinch(joy.getRawAxis(5));
      if (timer.get() > 0.55) {
        m_climb.moveClimb(joy.getRawAxis(5));
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //needs lock
    m_climb.moveClimb(0);
    m_climb.moveWinch(0);
    m_pneumatics.lockWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(joy.getRawAxis(5)) < 0.1;
  }
}
