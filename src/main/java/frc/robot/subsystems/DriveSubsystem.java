/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.util.Units;
import static frc.robot.Constants.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  WPI_VictorSPX m_leftSlave = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
  WPI_VictorSPX m_rightSlave = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);

  // The motors on the left side of the drive.
  // using follow mode so specify only a single motor
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMaster);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMaster);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Kinematics for drive
  private final SlewRateLimiter m_speedRateLimiter_Low = new SlewRateLimiter(
      ArcadeConstants.kSlewRateSpeedMetersPerSecond_Low);
  private final SlewRateLimiter m_speedRateLimiter_High = new SlewRateLimiter(
      ArcadeConstants.kSlewRateSpeedMetersPerSecond_High);
  private final SlewRateLimiter m_rotationRateLimiter = new SlewRateLimiter(
      ArcadeConstants.kSlewRateRotationSpeedRadiansPerSecond);
  private final ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private final SimpleMotorFeedforward m_feedForward_Low = new SimpleMotorFeedforward(DriveConstants.ksVolts_Low,
      DriveConstants.kvVoltSecondsPerMeter_Low, DriveConstants.kaVoltSecondsSquaredPerMeter_Low);
  private final SimpleMotorFeedforward m_feedForward_High = new SimpleMotorFeedforward(DriveConstants.ksVolts_High,
      DriveConstants.kvVoltSecondsPerMeter_High, DriveConstants.kaVoltSecondsSquaredPerMeter_High);

  public Supplier<Double> m_leftEncoderPosition;
  public Supplier<Double> m_leftEncoderRate;
  public Supplier<Double> m_rightEncoderPosition;
  public Supplier<Double> m_rightEncoderRate;
  public Supplier<Integer> m_leftEncoderPositionTicks;
  public Supplier<Integer> m_rightEncoderPositionTicks;
  public Supplier<Double> m_gyroAngleRadians;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private static int PIDIDX = 0;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private Boolean m_bUseVelocityForDrive = true;

  // Arcade drive modifiers
  private static boolean m_reverse = false;

  private final PneumaticSubsystem m_pneumaticSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final ColorWheelSubsystem m_colorWheel;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(PneumaticSubsystem pneumaticSubsystem, IntakeSubsystem intakeSubsystem,
      ColorWheelSubsystem colorWheelSubsystem) {

    m_pneumaticSubsystem = pneumaticSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_colorWheel = colorWheelSubsystem;

    // zeroing yaw seems to not work if AHRS was recently created, so sleep here
    // it's not terribly friendly, but we can afford half a second during robotInit
    try {
      Thread.sleep(500, 0);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    zeroHeading();
    m_rightMotors.setInverted(true);

    // before constructing DifferentialDriveOdometry, make sure encoders are reset to zero!
    resetEncoders();

    configureTalons();

    m_drive.setDeadband(0.01);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    resetOdometry(new Pose2d());
  }

  private void configureTalons() {

    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_leftSlave.configFactoryDefault();
    m_rightSlave.configFactoryDefault();

    m_leftMaster.setInverted(DriveConstants.kLeftTalonInverted);
    m_leftMaster.setSensorPhase(false);
    m_leftMaster.setNeutralMode(NeutralMode.Coast);
    m_leftMaster.configNeutralDeadband(DriveConstants.kTalonDeadband);

    m_leftSlave.setInverted(DriveConstants.kLeftTalonInverted);
    m_leftSlave.follow(m_leftMaster);
    m_leftSlave.setNeutralMode(NeutralMode.Coast);
    m_leftSlave.configNeutralDeadband(DriveConstants.kTalonDeadband);

    m_rightMaster.setInverted(DriveConstants.kRightTalonInverted);
    m_rightMaster.setSensorPhase(false);
    m_rightMaster.setNeutralMode(NeutralMode.Coast);
    m_rightMaster.configNeutralDeadband(DriveConstants.kTalonDeadband);

    m_rightSlave.setInverted(DriveConstants.kRightTalonInverted);
    m_rightSlave.follow(m_rightMaster);
    m_rightSlave.setNeutralMode(NeutralMode.Coast);
    m_rightSlave.configNeutralDeadband(DriveConstants.kTalonDeadband);

    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, 50);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, 50);

    m_leftEncoderPosition = ()
    -> m_leftMaster.getSelectedSensorPosition(PIDIDX) * DriveConstants.kEncoderDistancePerPulse;

    m_leftEncoderPositionTicks = ()
    -> m_leftMaster.getSelectedSensorPosition(PIDIDX);
    
    // multiply by 10 as CTRE velocity is units per 100ms and RamseteCommand expects meters per second
    m_leftEncoderRate = ()
    -> m_leftMaster.getSelectedSensorVelocity(PIDIDX) * DriveConstants.kEncoderDistancePerPulse * 10.0;

    m_rightEncoderPosition = ()
    -> m_rightMaster.getSelectedSensorPosition(PIDIDX) * DriveConstants.kEncoderDistancePerPulse;
    
    m_rightEncoderPositionTicks = ()
    -> m_rightMaster.getSelectedSensorPosition(PIDIDX);

    // multiply by 10 as CTRE velocity is units per 100ms and RamseteCommand expects meters per second
    m_rightEncoderRate = ()
    -> m_rightMaster.getSelectedSensorVelocity(PIDIDX) * DriveConstants.kEncoderDistancePerPulse * 10.0;
  
    m_leftMaster.config_kP(PIDIDX, DriveConstants.kpTalonDriveVel);
    m_rightMaster.config_kP(PIDIDX, DriveConstants.kpTalonDriveVel);
    m_leftMaster.config_kI(PIDIDX, 0.000);
    m_rightMaster.config_kI(PIDIDX, 0.000);
    m_leftMaster.config_kD(PIDIDX, DriveConstants.kdTalonDriveVel);
    m_rightMaster.config_kD(PIDIDX, DriveConstants.kdTalonDriveVel);
    // probably not needed as we are not using kI
    m_leftMaster.config_IntegralZone(PIDIDX, 3000);
    m_rightMaster.config_IntegralZone(PIDIDX, 3000);
    m_leftMaster.configClosedLoopPeakOutput(PIDIDX, 1.0);
    m_rightMaster.configClosedLoopPeakOutput(PIDIDX, 1.0);

    m_leftMaster.configMotionCruiseVelocity((int)(DriveConstants.kMaxSpeedMetersPerSecond_Low / DriveConstants.kEncoderDistancePerPulse / 10.0));
    m_rightMaster.configMotionCruiseVelocity((int)(DriveConstants.kMaxSpeedMetersPerSecond_Low / DriveConstants.kEncoderDistancePerPulse / 10.0));

    m_leftMaster.configMotionAcceleration((int)(DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low / DriveConstants.kEncoderDistancePerPulse / 10.0));
    m_rightMaster.configMotionAcceleration((int)(DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low / DriveConstants.kEncoderDistancePerPulse / 10.0));
    
    m_leftMaster.setSelectedSensorPosition(0, PIDIDX, 50);
    m_rightMaster.setSelectedSensorPosition(0, PIDIDX, 50);

    m_leftMaster.configOpenloopRamp(DriveConstants.kTalonOpenLoopRamp);
    m_rightMaster.configOpenloopRamp(DriveConstants.kTalonOpenLoopRamp);
    m_leftMaster.configVoltageCompSaturation(DriveConstants.kBatteryCompensationForTalon);
    m_rightMaster.configVoltageCompSaturation(DriveConstants.kBatteryCompensationForTalon);
    m_leftMaster.enableVoltageCompensation(true);
    m_rightMaster.enableVoltageCompensation(true);

    m_leftMaster.configVelocityMeasurementWindow(16);
    m_leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
    m_rightMaster.configVelocityMeasurementWindow(16);
    m_rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);

    /*  used if we want to persist to non-volatile storage on the Talon
    // make sure to assign all appropriate values
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.slot0.kP = DriveConstants.kpTalonDriveVel;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = DriveConstants.kdTalonDriveVel;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.closedloopRamp = DriveConstants.kTalonClosedLoopRamp;
    talonConfig.openloopRamp = DriveConstants.kTalonOpenLoopRamp;

    m_rightMaster.configAllSettings(talonConfig);
    m_leftMaster.configAllSettings(talonConfig);
    */
  }


  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  //NetworkTableEntry m_Pose = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Pose");
  NetworkTableEntry m_Heading = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Heading");

  @Override
  public void periodic() {

    m_drive.feed();

    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoderPosition.get(),
                      m_rightEncoderPosition.get());

    //for debug
    Pose2d currentPose = getPose();
    Translation2d translation = currentPose.getTranslation();
    double heading = currentPose.getRotation().getDegrees();

    m_xEntry.setDouble(translation.getX());
    m_yEntry.setDouble(translation.getY());
    m_Heading.setDouble(heading);

    //m_Pose.setString(currentPose.toString());
    //SmartDashboard.putString("Pose", currentPose.toString());
    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoderRate.get(), m_rightEncoderRate.get());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement from -1 to 1
   * @param rot the commanded rotation from -1 to 1
   */
  public void arcadeDrive(double fwd, double rot) {
    double vMetersPerSecond = 0.0;
    if (m_reverse) {
      fwd *= -1;
    }

    if (m_colorWheel.isServoUp()) {
      fwd *= 0.15;
      rot *= 0.5;
    }

    if (m_intakeSubsystem.isSpinning()) {
      fwd *= 0.5;
      rot *= 0.9;
    }

    if (m_bUseVelocityForDrive) {

      // if input is small, consider it 0 to prevent motor creep
      if (Math.abs(fwd) < DriveConstants.kFwdInputDeadband) {
        fwd = 0.0;
      }

      if (Math.abs(rot) < DriveConstants.kRotInputDeadband) {
        rot = 0.0;
      }

      if (m_pneumaticSubsystem.getCurrentGear() == GearIndicator.low) {
        vMetersPerSecond =  fwd * DriveConstants.kMaxSpeedMetersPerSecond_Low;
        m_chassisSpeeds.vxMetersPerSecond = m_speedRateLimiter_Low.calculate(vMetersPerSecond);
      }
      else if (m_pneumaticSubsystem.getCurrentGear() == GearIndicator.high) {
        vMetersPerSecond =  fwd * DriveConstants.kMaxSpeedMetersPerSecond_High;
        m_chassisSpeeds.vxMetersPerSecond = m_speedRateLimiter_High.calculate(vMetersPerSecond);
      }
  
      double vRadiansPerSecond = rot * DriveConstants.kMaxRotationSpeedRadiansPerSecond;

      m_chassisSpeeds.vyMetersPerSecond = 0.0;  // our bot can't go sideways
      m_chassisSpeeds.omegaRadiansPerSecond = -m_rotationRateLimiter.calculate(vRadiansPerSecond);

      DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(m_chassisSpeeds);
      tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }
    else {
        m_drive.arcadeDrive(fwd, rot);
    }
  }


NetworkTableEntry m_leftVelStats = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("leftVel");
NetworkTableEntry m_rightVelStats = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("rightVel");
NetworkTableEntry m_leftPosStats = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("leftPos");
NetworkTableEntry m_rightPosStats = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("rightPos");

  /**
   * Controls the left and right sides of the drive with velocity.
   *
   * @param leftVelocity  the left velocity to maintain
   * @param rightVelocity the right velocity to maintain
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {

    double leftFeedForwardVolts = 0.0;
    double rightFeedForwardVolts = 0.0;

    // Keep motor safety happy
    m_drive.feed();

    // Velocity is expressed in meters per second.  Acceleration units must also remain consistent (m/s^2)
    // a = (v1 - v0) / t
    // We are using a one second time period in our acceleration calculation
    // As such, acceleration value will be less than true demand (since this is velocity over 1 second
    // rather than 20ms of the rate of the command scheduler).  Using 20ms instead results in too high
    // a voltage calculation from feedforward.calculate as it expects much lower inputs based on actual
    // robot characteristics (ks, kv, ka).  Feed forward with actual velocity demand and acceleration
    // calculated this way may not be exact, but is sufficiently close and the P loop on the Talon
    // will take care of any error delta.
    double leftAccel  = (leftVelocity -  m_leftEncoderRate.get());
    double rightAccel = (rightVelocity - m_rightEncoderRate.get());

    if (m_pneumaticSubsystem.getCurrentGear() == GearIndicator.low) {
      leftFeedForwardVolts = m_feedForward_Low.calculate(leftVelocity,  leftAccel);
      rightFeedForwardVolts = m_feedForward_Low.calculate(rightVelocity, rightAccel);
    }
    else if (m_pneumaticSubsystem.getCurrentGear() == GearIndicator.high) {
      leftFeedForwardVolts = m_feedForward_High.calculate(leftVelocity,  leftAccel);
      rightFeedForwardVolts = m_feedForward_High.calculate(rightVelocity, rightAccel);
    }

    // calculate demand for feedforward based on voltage characterization values passed to SimpleMotorFeedForward
    // Talon expects encoder units change / 100ms, so convert meters/s to units/100ms.
    // Some platforms (such as RoboRio's ARM Cortex-A9) are slower for div than mul, so multiply instead.
    // Java compiler may be smart enough to convert divide to multiply for this constant, but use mul to be safe.
    double leftVelocityForTalon =  (leftVelocity  / DriveConstants.kEncoderDistancePerPulse) * 0.1d; 
    double rightVelocityForTalon = (rightVelocity / DriveConstants.kEncoderDistancePerPulse) * 0.1d; 

    m_leftVelStats.setNumber(leftVelocityForTalon);//leftFeedForwardVolts);//leftVelocityForTalon);
    m_rightVelStats.setNumber(rightVelocityForTalon);//rightFeedForwardVolts);//rightVelocityForTalon);

    // Use Talon closed-loop control for velocity with calculated feedforward.
    // Velocities are in encoder units/100ms.
    // The last argument to Talon.set (feedforward) is an arbitrary value (-1 to 1).  Scale by nominal battery voltage rather than
    // real-time battery voltage as we have set the Talon to compensate for voltage.
    m_leftMaster.set( ControlMode.Velocity, leftVelocityForTalon,  DemandType.ArbitraryFeedForward, leftFeedForwardVolts  / DriveConstants.kNominalBatteryVoltage);
    m_rightMaster.set(ControlMode.Velocity, rightVelocityForTalon, DemandType.ArbitraryFeedForward, rightFeedForwardVolts / DriveConstants.kNominalBatteryVoltage);
  }

  /**
   * Controls the left and right sides of the drive with velocity.
   *
   * @param leftPositionForTalon  the left position to attain in ticks
   * @param rightPositionForTalon the right position to attain in ticks
   */
  public int tankDriveToPosition(double leftPositionForTalon, double rightPositionForTalon) {

    m_leftMaster.set(ControlMode.MotionMagic, leftPositionForTalon);
    m_rightMaster.set(ControlMode.MotionMagic, rightPositionForTalon);

    return m_leftMaster.getClosedLoopError(PIDIDX);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoderPosition.get() + m_rightEncoderPosition.get()) * 0.5d;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   * If calling zeroHeading, make sure pose is also reset or it will appear
   * that the robot has changed position.
   */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Set wehether teleop uses encoders or not.
   *
   * @param bVelMode If true, use velocity mode.  If false, use 0..1 input to differential drive's arcade drive
   */
  public void setTeleopDriveVelocityMode(Boolean bVelMode) {
    m_bUseVelocityForDrive = bVelMode;
  }


  public Pose2d calcEndPose(double distanceToTargetInches) {
    Pose2d currentPose = getPose();
    Pose2d targetPose;

    double distanceX;
    double distanceY;
    double desiredX;
    double desiredY;
    double angle;
    
    // robot will be facing away from target
    // angle of robot should be 0 to 90 (on right side of target) or 0 to -90 (left side of target) as positive is counter clockwise
    // interior angle of triangle made with robot, target, and normal will be 90-getHeading or 90+getHeading

    angle = currentPose.getRotation().getDegrees();

    //System.out.println("angle: " + angle);

    if (Math.abs(angle) > 90) {
      System.out.println("we have a problem -- angle is too large!  Are we facing the wrong direction?");
      return currentPose;
    }

    distanceX = Units.inchesToMeters(Math.sin(Math.toRadians(90 - angle)) * distanceToTargetInches);
    distanceY = Units.inchesToMeters(Math.cos(Math.toRadians(90 - angle)) * distanceToTargetInches);

    System.out.print("distanceX: " + distanceX);
    System.out.print("distanceY: " + distanceY);

    // add a bit of distance so we are in front of target.
    // cos of angle > 90 is negative, which will then get added to desiredY
    // which is correct position as negative value indicates we are to left of target
    desiredX = currentPose.getTranslation().getX() - distanceX + 0.6;
    desiredY = currentPose.getTranslation().getY() - distanceY;

    //System.out.print("desiredX: " + desiredX);
    //System.out.print("desiredY: " + desiredY);

    targetPose = new Pose2d(desiredX, desiredY, Rotation2d.fromDegrees(0));

    System.out.println("Target Distance (in): " + distanceToTargetInches);
    System.out.println("Cur Pose: " + currentPose.toString());
    System.out.println("Target Pose: " + targetPose.toString());

    return targetPose;
  }

  public Translation2d calcMidPoint(Pose2d startpoint, Pose2d endpoint) {
    Translation2d midpoint;
    double x;
    double y;

    x = endpoint.getTranslation().getX() + (startpoint.getTranslation().getX() - endpoint.getTranslation().getX()) / 2; 
    y = endpoint.getTranslation().getY() + (startpoint.getTranslation().getY() - endpoint.getTranslation().getY()) / 2;

    midpoint = new Translation2d(x, y);

    System.out.println("Mid: " + midpoint.toString());

    return midpoint;
  }

  public boolean isDrivingReversed() {
    return m_reverse;
  }

  public void setReverseDrive(boolean bReverse) {
    m_reverse = bReverse;
  }
}