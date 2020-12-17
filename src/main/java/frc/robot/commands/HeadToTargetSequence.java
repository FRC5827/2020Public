/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class 
HeadToTargetSequence extends SequentialCommandGroup {

  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelight;


  /**
   * Create a new DriveToTarget command.
   * @param drive The DriveSubsystem this command will run on
   * @param limelight The LimeLightSubsystem this command will run on
   * @param pneumatics The PneumaticSubsystem this command will run on
   */
  public HeadToTargetSequence(DriveSubsystem drive, LimelightSubsystem limelight, PneumaticSubsystem pneumatics, ConveyorSubsystem conveyor) {

    m_limelight = limelight;
    m_driveSubsystem = drive;

    // Sequence:
    // 1. Downshift and turn towards target using Limelight
    // 2. Determine distance to target based on Limelight
    // 3. Determine angle relative to straight line of target plane (gyro/Pose data -- assumes we were zeroed from the start)
    // 4. Based on distance to target and angle, generate trajectory to target minus an offset using current pose and desired final pose

    addCommands(
      new InstantCommand(() -> limelight.setPipeline(1)),
      new ShiftDown(pneumatics),
      new WaitCommand(0.15),
      new TurnToTarget(drive, limelight),
      new DriveToTarget(drive, limelight)
      //new DumpBalls(conveyor, .5)
    );
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_limelight.setPipeline(0);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }
  
}

