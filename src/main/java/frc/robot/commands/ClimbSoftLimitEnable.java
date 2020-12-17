package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;


/**
 * An example command that uses an example subsystem.
 */
public class ClimbSoftLimitEnable extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ClimbSubsystem m_climb;
  private double m_setpoint;

  /**
   * Creates a new ClimbPID.
   *
   * @param climb The climb used by this command.
   */
  public ClimbSoftLimitEnable(ClimbSubsystem climb) {
    m_climb = climb;
    // Use addRequirements() here to declare climb dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.overrideLimit(true);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
