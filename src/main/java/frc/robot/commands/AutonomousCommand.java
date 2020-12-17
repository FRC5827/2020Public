
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A complex auto command sequence.
 */
public class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param order    The order to exec
   * @param commands The list of commands to use from
   */
  public AutonomousCommand(int[] order, Command[] commands) {
    int length = order.length;
    for (int i = 0; i < length; i++) {
      System.out.println("length is " + order.length + " | index is " + i);
      System.out.println(commands[order[i]].toString());
      addCommands(commands[order[i]]);
    }
  }

}