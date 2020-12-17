/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoOrder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeIndicator;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ButtonBoardIDs;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.commands.*;
import frc.robot.resources.TrajectoryModifier;


import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
	private final LimelightSubsystem m_limelight = new LimelightSubsystem();
	private final ColorWheelSubsystem m_colorWheel = new ColorWheelSubsystem();
	private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
	private final IntakeSubsystem m_intake = new IntakeSubsystem(m_pneumaticSubsystem, m_conveyor);
	private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_pneumaticSubsystem, m_intake, m_colorWheel);
	private SendableChooser<Boolean> m_velocityDriveSelector;
	private SendableChooser<Integer> autoSelector;

//	private AddressableLED m_led = new AddressableLED(Constants.LEDConstants.kLEDPort);
//	private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
	
	private final ClimbSubsystem m_climb = new ClimbSubsystem();
	//private final ClimbPID m_autoCommand = new ClimbPID(m_climb);
	private final HookSubsystem m_hook = new HookSubsystem();

	// The driver's controller
	//Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

	// Button Board
	Joystick m_buttonBoard = new Joystick(ButtonBoardIDs.kButtonBoardPort);

	// Definitions of the various different autonomous commands go here

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		LiveWindow.disableAllTelemetry();

		// Configure the button bindings
		configureButtonBindings();
		
		// Create camera server for usb camera
		CameraServer.getInstance().startAutomaticCapture();

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		m_robotDrive.setDefaultCommand(
				// A split-stick arcade command, with forward/backward controlled by the left
				// hand, and turning controlled by the right.
				new RunCommand(() -> m_robotDrive.arcadeDrive(
						(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2)),
						m_driverController.getRawAxis(0)), m_robotDrive));
		//m_climb.setDefaultCommand(new ClimbMove(m_climb, m_driverController));
		m_conveyor.setDefaultCommand(new ConveyorSensor(m_conveyor));

		TrajectoryModifier.genAllTrajectories();	
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

		// Drive at half speed when the right bumper is held
		// Pneumatics
		/*
		 * new JoystickButton(m_driverController, Button.kBumperRight.value)
		 * .whenPressed(new ShiftUp(m_pneumaticSubsystem)); new
		 * JoystickButton(m_driverController, Button.kBumperLeft.value) .whenPressed(new
		 * ShiftDown(m_pneumaticSubsystem)); new JoystickButton(m_driverController,
		 * Button.kA.value) .whenPressed(new ServoUp(servo)); new
		 * JoystickButton(m_driverController, Button.kA.value) .whenPressed(new
		 * ServoDown(servo));
		 */

		 /*
		new JoystickButton(m_driverController, Button.kB.value).whenPressed(new ColorServoCommand(m_colorWheel, true));
		new JoystickButton(m_driverController, Button.kA.value).whenPressed(new TurnWheelCommand(m_colorWheel, 8));
		new JoystickButton(m_driverController, Button.kY.value).whenPressed(new SpecificColorCommand(m_colorWheel));
		new JoystickButton(m_driverController, Button.kX.value).whenPressed(new ColorServoCommand(m_colorWheel, false));
		*/

		//Limelight
		new JoystickButton(m_driverController, Button.kX.value).whenPressed(new HeadToTargetSequence(m_robotDrive, m_limelight, m_pneumaticSubsystem, m_conveyor).withInterrupt(() -> Math.abs(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2)) > .04));

		//Reverse
		new JoystickButton(m_driverController, Button.kStickLeft.value).whenPressed(new Reverse(m_robotDrive, m_intake));

		//Intake
		new JoystickButton(m_driverController, Button.kA.value)
			.whenPressed(new IntakeSpin(m_intake, m_conveyor, m_driverController));
			//.whenReleased(new Intake(m_intake, m_robotDrive, m_conveyor, m_driverController, false));

		//Conveyor Belt Dump
		new JoystickButton(m_driverController, Button.kB.value).whenPressed(new ConveyorOut(m_conveyor, 1, m_driverController));

		//new JoystickButton(m_driverController, Button.kB.value).whenPressed(new ColorServoCommand(m_colorWheel, m_robotDrive, m_driverController));

		//Drive invert
		//new JoystickButton(m_driverController, Button.kY.value).whenPressed(new )


		//Climb and Winch
		new Trigger(() -> (Math.abs(m_driverController.getRawAxis(5)) > 0.3)
			&& m_buttonBoard.getRawButton(Constants.ButtonBoardIDs.kClimbEnable))
				.whenActive(new ClimbMove(m_climb, m_pneumaticSubsystem, m_intake, m_driverController));


		//Hook
		new JoystickButton(m_driverController, Button.kStart.value)
			.whenPressed(new SetServo(m_hook, true))
			.whenReleased(new SetServo(m_hook, false));

		
		//Shifter
		new JoystickButton(m_driverController, Button.kBumperLeft.value).whenPressed(new ShiftDown(m_pneumaticSubsystem));
		new JoystickButton(m_driverController, Button.kBumperRight.value).whenPressed(new ShiftUp(m_pneumaticSubsystem));

		// Button Board:
		// Intake and Conveyor
		
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kIntakeDirectionOut)
			.whenPressed(new IntakeButtonBoard(m_intake, m_buttonBoard, -0.25));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kManualConveyorOut).whenPressed(new InverseConveyor(m_conveyor, -1, m_buttonBoard));

		// Control Panel
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kManualControlPanelLeft)
			.whenPressed(new ControlPanelManual(m_colorWheel, m_buttonBoard, ButtonBoardIDs.kManualControlPanelLeft, false));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kManualControlPanelRight)
			.whenPressed(new ControlPanelManual(m_colorWheel, m_buttonBoard, ButtonBoardIDs.kManualControlPanelRight, true));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kColorServoUp)
			.whenPressed(new ColorServoCommand(m_colorWheel, m_buttonBoard, ButtonBoardIDs.kColorServoUp, true))
			.whenReleased(new ColorServoCommand(m_colorWheel, m_buttonBoard, ButtonBoardIDs.kColorServoUp, false));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kSpecificColor).whenPressed(new SpecificColorCommand(m_colorWheel));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kColorPosition).whenPressed(new TurnWheelCommand(m_colorWheel, 7.5));

		// Climb, Winch and Hook
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kWinchIn)
			.whenPressed(new Winch(m_climb, m_buttonBoard, 0.5, ButtonBoardIDs.kWinchIn));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kWinchOut)
			.whenPressed(new Winch(m_climb, m_buttonBoard, -0.5, ButtonBoardIDs.kWinchOut));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kHookOpen)
			.whenPressed(new SetServo(m_hook, true))
			.whenReleased(new SetServo(m_hook, false));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kLiftUp)
			.whenPressed(new Lift(m_climb, m_buttonBoard, -0.5, ButtonBoardIDs.kLiftUp));
		new JoystickButton(m_buttonBoard, ButtonBoardIDs.kLiftDown)
			.whenPressed(new Lift(m_climb, m_buttonBoard, 0.5, ButtonBoardIDs.kLiftDown));
		

		// Override
		// to do: override

	}

	public void initChooser() {
		autoSelector = new SendableChooser<Integer>();
		autoSelector.setDefaultOption("Cross line", 0);
		autoSelector.setDefaultOption("Dump Balls", 2);
		autoSelector.addOption("Trench Run", 1);
		autoSelector.addOption("Steal Balls", 3);
		Shuffleboard.getTab("Autonomous").add("Autonomous Picker", autoSelector);

		m_velocityDriveSelector = new SendableChooser<Boolean>();
		m_velocityDriveSelector.addOption("False", false);
		m_velocityDriveSelector.setDefaultOption("True", true);
		Shuffleboard.getTab("Autonomous").add("Use encoders for teleop drive", m_velocityDriveSelector);
	}

	public void initGyroAndResetPose() {
		m_robotDrive.zeroHeading();
		m_robotDrive.resetOdometry(new Pose2d());

	}

	public void setTeleDriveMode() {
		m_robotDrive.setTeleopDriveVelocityMode(m_velocityDriveSelector.getSelected());
	}


	public int[] getAutoOrder() {
		return AutoOrder.autos[autoSelector.getSelected()];
	}

	public int getSelectedIndex() {
		return autoSelector.getSelected();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getReverseAutonomousCommand0() {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		if (!TrajectoryModifier.getAutoListReverse0()[autoSelector.getSelected()].isReversed()) {
			return new ShiftUp(m_pneumaticSubsystem);
		}

		m_pneumaticSubsystem.shiftUp();

		RamseteCommand ramseteCommand_r0 = new RamseteCommand(TrajectoryModifier.chooseReverseTrajectory0(autoSelector),
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
				  m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
				  //m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
				  //m_leftReference.setNumber(leftMetersPerSec);
		  
				  //m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
				  //m_rightReference.setNumber(rightMetersPerSec);
				},
		   m_robotDrive);

		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, -3.048);
		return ramseteCommand_r0.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));

	}

	public Command getAutonomousCommand() {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		System.out.println("AUTONOMOUS COMMAND");

		// set current robot position at postion 0, with relative heading of 0
		// which should match trajectory provided below

		//m_pneumaticSubsystem.ShiftUp();

		Trajectory trajectoryToFollow = TrajectoryModifier.chooseTrajectory(autoSelector);
		System.out.print("Time it will take to traverse trajectory ");
		System.out.println(trajectoryToFollow);
		System.out.println(trajectoryToFollow.getTotalTimeSeconds());

		RamseteCommand ramseteCommand = new RamseteCommand(TrajectoryModifier.chooseTrajectory(autoSelector),
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
					m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
			
					//m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
					//m_leftReference.setNumber(leftMetersPerSec);
			
					//m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
					//m_rightReference.setNumber(rightMetersPerSec);
				  },
			 m_robotDrive);

  
		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, 0.5);
		ParallelRaceGroup parallelRace = new ParallelRaceGroup(new ConveyorSensor(m_conveyor), new IntakeSpinPerpetual(m_intake, m_conveyor));
		ParallelDeadlineGroup deadlineGroup = new ParallelDeadlineGroup(ramseteCommand, parallelRace);
		return deadlineGroup.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
		//ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
		//return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
		//return TurnWheelCommand(m_colorWheel ,2);
	}

	public Command getSlowAutonomousCommand() {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		//System.out.println("AUTONOMOUS COMMAND");

		// set current robot position at postion 0, with relative heading of 0
		// which should match trajectory provided below

		if (!TrajectoryModifier.getAutoListSlow()[autoSelector.getSelected()].isReversed()) {
			return new ShiftUp(m_pneumaticSubsystem);
		}

		m_pneumaticSubsystem.shiftDown();

		Trajectory trajectoryToFollow = TrajectoryModifier.chooseSlowTrajectory(autoSelector);
		System.out.print("Time it will take to traverse trajectory ");
		System.out.println(trajectoryToFollow);
		System.out.println(trajectoryToFollow.getTotalTimeSeconds());
		//Trajectory initialTrajectory = TrajectoryModifier.
		RamseteCommand ramseteCommand = new RamseteCommand(TrajectoryModifier.chooseSlowTrajectory(autoSelector),
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
					m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
			
					//m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
					//m_leftReference.setNumber(leftMetersPerSec);
			
					//m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
					//m_rightReference.setNumber(rightMetersPerSec);
				  },
			 m_robotDrive);

  
		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, 0.5);
		ParallelRaceGroup parallelRace = new ParallelRaceGroup(new ConveyorSensor(m_conveyor), new IntakeSpinPerpetual(m_intake, m_conveyor));
		ParallelDeadlineGroup deadlineGroup = new ParallelDeadlineGroup(ramseteCommand, parallelRace);
		return deadlineGroup.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));//ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
		//return TurnWheelCommand(m_colorWheel ,2);
	}

	public Command getReverseAutonomousCommand1() {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		if (!TrajectoryModifier.getAutoListReverse1()[autoSelector.getSelected()].isReversed()) {
			return new ShiftUp(m_pneumaticSubsystem);
		}

		m_pneumaticSubsystem.shiftUp();

		RamseteCommand ramseteCommand_r2 = new RamseteCommand(TrajectoryModifier.chooseReverseTrajectory1(autoSelector),
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
					m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
			
					//m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
					//m_leftReference.setNumber(leftMetersPerSec);
			
					//m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
					//m_rightReference.setNumber(rightMetersPerSec);
				  },
			 m_robotDrive);

		// Run path following command, then stop at the end.
		return ramseteCommand_r2;
	}

	public Command getIntakeCommand() {
		return new IntakeSpinPerpetual(m_intake, m_conveyor);
	}

	public Command getConveyorCommand() {
		return new ConveyorSensor(m_conveyor);
	}

	public Command getDumpCommand() {
		return new DumpBalls(m_conveyor, 0.7);
	}

	public Command getDumpCommand1() {
		return new DumpBalls(m_conveyor, 0.7);
	}
  
	public Command getTurnCommand(double degreesToTurn) {
		return new TurnDegrees(m_robotDrive, false, degreesToTurn);
	}

	public void lockWinch() {
		m_pneumaticSubsystem.lockWinch();
	}

	public void unlockWinch() {
		m_pneumaticSubsystem.unlockWinch();
	}

	public void intakeUp() {
		m_intake.setIntake(IntakeIndicator.up);
	}

	/*
	public void initLEDs() {
		m_led.setLength(m_ledBuffer.getLength());
	}

	public AddressableLED getLED() {
		return m_led;
	}

	public AddressableLEDBuffer getLEDBuffer() {
		return m_ledBuffer;
	}

	public void setLEDColor(int r, int g, int b) {
		for (var i = 0; i < m_ledBuffer.getLength(); i++) {
			m_ledBuffer.setRGB(i,r, g, b);
		}
		m_led.setData(m_ledBuffer);
	}

	public void updateLEDs() {
		if(m_robotDrive.isDrivingReversed()) {
			setLEDColor(255, 0, 0); //red
		} else {
			setLEDColor(0, 255, 0); //green
		}
	}
	*/
}
