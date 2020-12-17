/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class ConveyorbeltId {
    public static final int ConveyorbeltMotorID = 7;
    public static final int JoystickIDSpeed = 2;
    public static final int bottomRightSensorID = 9;
    public static final int bottomLeftSensorID = 8;
    public static final int topSensorID = 0;
    public static final int JoystickIDRollaRoda = 3;
  }

  public static final int kSlotIdx = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
   * we just want the primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  /**
   * Choose based on what direction you want to be positive, this does not affect
   * motor invert.
   */
  public static boolean kMotorInvert = false;

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki,
   * kd, kf, izone, peak output);
   */

  public class kGains {
    public static final double kF = 0;
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class ClimbConstants {
    public static final int kWinch1 = 5;
    public static final int kWinch2 = 8;
    public static final int kClimbMotor = 3;
  }

  public static final class ColorWheelConsants {
    public static final int kServey = 0;
    public static final int kWheelMotor = 10;
  }

  public static final class HookConstant {
    public static final int kServo = 9;
  }

  public static final class IntakeConstant {
    public static final int kMotor = 9;
    public static final double kMotorSpeed = 0.5;
  }

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 6;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 4;

    public static final int kLeftEncoderPort = 0;
    public static final int kRightEncoderPort = 0;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 30720;
    public static final double kWheelDiameterMeters = 0.153;//0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    public static final double kNominalBatteryVoltage = 12.0;
    public static final double kBatteryCompensationForTalon = 11.0;

    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *our* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for our specific robot.

    public static final double ksVolts_Low = 1.05;
    // voltage to hold a constant velocity
    public static final double kvVoltSecondsPerMeter_Low = 4.91;
    // voltage needed to generate given acceleration
    public static final double kaVoltSecondsSquaredPerMeter_Low = 0.786;

    // voltage to overcome static friction
    public static final double ksVolts_High = 1.27;
    // voltage to hold a constant velocity
    public static final double kvVoltSecondsPerMeter_High = 2.36;
    // voltage needed to generate given acceleration
    public static final double kaVoltSecondsSquaredPerMeter_High = 1.08;

    // As above, this must be tuned for our drive!
    // public static final double kPDriveVel = 19.7;
    public static final double kPDriveVel = 15; // value from characterization when slave is selected in UI

    public static final double kTrackwidthMeters = 0.6961;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double kDistancePerDegreeOfTurn = (Math.PI * kTrackwidthMeters) / 360.0;

    // this value should be set in the vicinity of what the actual robot top speed
    // is
    public static final double kMaxSpeedMetersPerSecond_Low = 2.3;
    public static final double kMaxAccelerationMetersPerSecondSquared_Low = 2.0;

    public static final double kMaxSpeedMetersPerSecond_High = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared_High = 3.0;

    public static final double kMaxRotationSpeedRadiansPerSecond = Units.degreesToRadians(270);
    public static final double kMaxCentripetalAccelerationMetersPerSecondSq = 0.6;

    public static final double kpDriveVel = 15.0; // value from characterization

    public static final double kpTalonDriveVel = 0.15;// .025;//0.15; // ideallly tune with phoenix tuner
    public static final double kdTalonDriveVel = 1.5;// 2.0;
    public static final double kTalonOpenLoopRamp = 0.33; // time to ramp or slew to max
    public static final double kTalonDeadband = 0.02; // less than this output will not drive motor
    public static final boolean kLeftTalonInverted = true;
    public static final boolean kRightTalonInverted = false;

    public static final double kFwdInputDeadband = 0.04;
    public static final double kRotInputDeadband = 0.10;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class AutoDefinitions {

    // Cross Line

    public static List<Translation2d> autoLinePoints = List.of(new Translation2d(1, 0));
    public static Pose2d[] autoLineEndPoints = { new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(2, 0, new Rotation2d(0)) };

    // TRENCH RUN

    public static List<Translation2d> trenchRunPointsReverse0 = List.of(new Translation2d(-1.5, 0));
    public static Pose2d[] trenchRunEndPointsReverse0 = { new Pose2d(0, 0, new Rotation2d(6.28)),
        new Pose2d(-2.1, 0, new Rotation2d(6.28)) };

    public static List<Translation2d> trenchRunPoints = List.of();
    // public static List<Translation2d> trenchRunPoints = List.of(new
    // Translation2d(0, 0));
    public static Pose2d[] trenchRunEndPoints = { new Pose2d(-2.1, 0, new Rotation2d(0)),
        new Pose2d(2.70, 1.73, new Rotation2d(0)) };

    public static List<Translation2d> trenchRunPointsSlow = List.of();
    // public static List<Translation2d> trenchRunPoints = List.of(new
    // Translation2d(0, 0));
    public static Pose2d[] trenchRunEndPointsSlow = { new Pose2d(2.70, 1.73, new Rotation2d(0)),
        new Pose2d(4.4/*5.2*/, 1.73, new Rotation2d(0)) };

    public static List<Translation2d> trenchRunPointsReverse = List.of(new Translation2d(1, 1.73));
    public static Pose2d[] trenchRunEndPointsReverse = { new Pose2d(4.4/*5.2*/, 1.73, new Rotation2d(6.28)),
        new Pose2d(-2.3, 0, new Rotation2d(6.28)) };

    // DUMP

    public static List<Translation2d> dumpPointsReverse0 = List.of(new Translation2d(-1.524, 0));
    public static Pose2d[] dumpEndPointsReverse0 = { new Pose2d(0, 0, new Rotation2d(6.28)),
        new Pose2d(-2.1, 0, new Rotation2d(6.28)) };

    public static List<Translation2d> dumpPoints = List.of();
    public static Pose2d[] dumpEndPoints = { new Pose2d(-3.048, 0, new Rotation2d(0)),
        new Pose2d(4, 1.47, new Rotation2d(0)) };


    // STEAL BALLS

    public static List<Translation2d> stealPoints = List.of();
    public static Pose2d[] stealEndPoints = { new Pose2d(0,0, new Rotation2d(0)), new Pose2d(2, 0, new Rotation2d(0.785)) };

    public static List<Translation2d> stealPointsReverse = List.of();
    public static Pose2d[] stealEndPointsReverse = {new Pose2d(2, 0, new Rotation2d(0.785)), new Pose2d(-2, -1.524, new Rotation2d(0))};
  }

  public static final class AutoOrder {
    public static int[] lineOrder = { 1 };
    public static int[] dumpOrder = { 0, 3, 1 };
    public static int[] trenchOrder = { 0, 3, 1, 5, 2, 4}; // should be like 1, 0, 2
    public static int[] stealOrder = {1, 2, 3};

    public static int[][] autos = { lineOrder, trenchOrder, dumpOrder, stealOrder };
  }

  public final class LimelightVals {
    public static final double TARGET_HEIGHT = 89.75; // height of center of target bounding box from ground in inches -- from game manual
    public static final double CAMERA_HEIGHT = 26.5; // height of camera from ground in inches
    public static final double TESTING_TARGET_DISTANCE = 174; // temporary distance between camera and target for calc
                                                              // the camera angle
    public static final double TESTING_CAMERA_ANGLE = 34.74; // temporary camera angle
  }

  public static final class TurnPIDConstants {
    public static final double kpTurnRio = 0.050;
    public static final double kiTurnRio = 0.000;
    public static final double kdTurnRio = 0.0015;
  }

  public static final class ArcadeConstants {
    public static final double kSlewRateSpeedMetersPerSecond_Low = DriveConstants.kMaxSpeedMetersPerSecond_Low
        / DriveConstants.kTalonOpenLoopRamp; // slew rate to reach max
    public static final double kSlewRateSpeedMetersPerSecond_High = DriveConstants.kMaxSpeedMetersPerSecond_High
        / DriveConstants.kTalonOpenLoopRamp; // slew rate to reach max
    public static final double kSlewRateRotationSpeedRadiansPerSecond = DriveConstants.kMaxRotationSpeedRadiansPerSecond
        / DriveConstants.kTalonOpenLoopRamp; // max degrees per second rate of change
  }

  public static final class PneumaticConstants {
    // Pneumatics
    public static final int PCM_ID = 0;

    public static final int SHIFTER_FORWARD_CHANNEL = 2;
    public static final int SHIFTER_BACKWARD_CHANNEL = 3;

    public static final int INTAKE_FORWARD_CHANNEL = 0;
    public static final int INTAKE_BACKWARD_CHANNEL = 1;

    public static final int WINCH_LOCK_ID = 4;
  }

  public static final class ButtonBoardIDs {
    // Button Board port
    public static final int kButtonBoardPort = 1;

    // Intake and Conveyor
    public static final int kWinchOut = 2;
    public static final int kWinchIn = 1;
    public static final int kIntakeDirectionOut = 3;
    public static final int kManualConveyorOut = 4;

    // Control Panel
    public static final int kManualControlPanelLeft = 5;
    public static final int kManualControlPanelRight = 6;
    public static final int kColorServoUp = 7;
    public static final int kSpecificColor = 9;
    public static final int kColorPosition = 8;

    // Climb and Hook
    public static final int kActivateClimb = 10;
    public static final int kHookOpen = 13;
    public static final int kLiftUp = 12;
    public static final int kLiftDown = 11;

    // Override
    public static final int kOverride = 14;
    public static final int kClimbEnable = 15;
  }

  public static final class LEDConstants {
    //public static final int kLEDPort = 9;
    //public static final int kLEDLength = 60;
  }

  public static enum GearIndicator {
    low, high
  }

  public static enum IntakeIndicator {
    up, down
  }

  public static enum ColorServoState {
    up, down
  }

}
