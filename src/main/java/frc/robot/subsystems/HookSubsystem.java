/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HookSubsystem extends SubsystemBase {
    
    // Create hook
    private Servo hook = new Servo(Constants.HookConstant.kServo);

    public HookSubsystem() {
        moveHook(false);
    }

    /**
     * 
     * Move hook to a set position
     * 
     * @param open True to open, false to close
     * 
     */
    public void moveHook(boolean open) {
        if (open) {
            hook.set(0.77);
        } else {
            hook.set(0.02);
        }
    }
}
