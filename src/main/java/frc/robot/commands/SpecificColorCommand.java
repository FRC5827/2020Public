/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSubsystem;

import edu.wpi.first.wpilibj.DriverStation;

public class SpecificColorCommand extends CommandBase {
  
  private final ColorWheelSubsystem m_subsystem;
  private String gameData = "";
  private String target;
  private int direction;

  public SpecificColorCommand(ColorWheelSubsystem subsystem) {
    m_subsystem = subsystem;
    target = "";
    direction = 1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    direction = 1;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        // there is a two color offset
        case 'B' :
          target = "Red";
          break;
        case 'G' :
          target = "Yellow";
          break;
        case 'R' :
          target = "Blue";
          break; 
        case 'Y' :
          target = "Green";
          break;
        default:
          target = "Unknown";
          direction = 0;
          System.out.println("It broke");
          break;
      }
    }
    // reverse direction for fastest path to color
    switch (m_subsystem.checkColor()) {
      case "Red" :
        if(target.equals("Green")) {
          direction = -1;
        }
        break;
      case "Yellow" :
        if(target.equals("Red")) {
          direction = -1;
        }
        break;
      case "Blue" :
        if(target.equals("Yellow")) {
          direction = -1;
        }
        break;
      case "Green" :
        if(target.equals("Blue")) {
          direction = -1;
        }
        break;
      default:
        direction = 0;
        System.out.println("It broke");
        break;
    }
    System.out.println("direction: " + direction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.turnWheelSlow(direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.turnMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.checkColor().equals(target);
  }
}