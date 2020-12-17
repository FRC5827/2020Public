/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {

    // Define motors
    private WPI_VictorSPX winch1 = new WPI_VictorSPX(Constants.ClimbConstants.kWinch1);
    private WPI_TalonSRX climbMotor = new WPI_TalonSRX(Constants.ClimbConstants.kClimbMotor);
    private WPI_VictorSPX winch2 = new WPI_VictorSPX(Constants.ClimbConstants.kWinch2);

    private final double kClimbTick2Feet = 7.5 / (4096 * Math.PI * 12);

    public ClimbSubsystem() {
        climbMotor.configFactoryDefault();
        winch1.configFactoryDefault();
        winch2.configFactoryDefault();
        // encoder setup
        configureClimb();

        resetEncoders();

        // for test
        // slave setup
        climbMotor.setInverted(false);

        climbMotor.setSensorPhase(false);

    }

    public boolean isFinished(int dist) {
        return (climbMotor.getSelectedSensorPosition() * kClimbTick2Feet < dist);
    }

    public void resetEncoders() {
        climbMotor.setSelectedSensorPosition(0, 0, 10);

    }

    /**
     * 
     * Configure the climb at the start, including resetting encoders and setting
     * PID
     * 
     */
    public void configureClimb() {
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        climbMotor.configAllowableClosedloopError(0, 100, Constants.kTimeoutMs);
        climbMotor.configForwardSoftLimitThreshold(0);
        climbMotor.configForwardSoftLimitEnable(true);
        climbMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        climbMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        climbMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        climbMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        winch1.setNeutralMode(NeutralMode.Brake);
        winch2.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climb encoder", climbMotor.getSelectedSensorPosition());
        // This method will be called once per scheduler run
    }

    public void setToMin(double setpoint) {
        climbMotor.set(ControlMode.Position, setpoint / kClimbTick2Feet);
    }

    /**
     * moveClimb moves the climb either up and down based upon the inputted speed
     * 
     * @param speed The speed that the motors move at, between -1 and 1
     */
    public void moveClimb(double speed) {
        if (speed > 0) {
            // preClimb(true);
            climbMotor.set(ControlMode.PercentOutput, speed * 0.8);
        } else {
            // preClimb(false);
            climbMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void moveWinch(double speed) {
        if (speed > 0) {
            winch1.set(ControlMode.PercentOutput, speed * 0.5);
            winch2.set(ControlMode.PercentOutput, speed * 0.5);
        } else {
            winch1.set(ControlMode.PercentOutput, speed * 0.8);
            winch2.set(ControlMode.PercentOutput, speed * 0.8);
        }

    }

    public void setCoastMode() {
        climbMotor.setNeutralMode(NeutralMode.Coast);
        winch1.setNeutralMode(NeutralMode.Coast);
        winch2.setNeutralMode(NeutralMode.Coast);
    }

    public void setBrakeMode() {
        climbMotor.setNeutralMode(NeutralMode.Brake);
        winch1.setNeutralMode(NeutralMode.Brake);
        winch2.setNeutralMode(NeutralMode.Brake);
    }

    public void overrideLimit(Boolean yes) {
        climbMotor.configReverseSoftLimitEnable(yes);
    }

}
