package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorServoState;
import edu.wpi.first.wpilibj.Servo;

public class ColorWheelSubsystem extends SubsystemBase {

    private final Servo servey = new Servo(Constants.ColorWheelConsants.kServey);
    private final WPI_VictorSPX motor = new WPI_VictorSPX(Constants.ColorWheelConsants.kWheelMotor);
    private final I2C.Port porty = I2C.Port.kOnboard;
    private final ColorSensorV3 sensy = new ColorSensorV3(porty);
    private final ColorMatch matchy = new ColorMatch();

    private Constants.ColorServoState m_servoState;

    public final Color kBlueTarget = ColorMatch.makeColor(0.251, 0.475, 0.278);
    public final Color kGreenTarget = ColorMatch.makeColor(0.285, 0.532, 0.177);
    public final Color kRedTarget = ColorMatch.makeColor(0.571, 0.345, 0.082);
    public final Color kYellowTarget = ColorMatch.makeColor(0.411, 0.491, 0.093);

    public ColorWheelSubsystem() {
        motor.configFactoryDefault();
        motor.configOpenloopRamp(0.4);
        matchy.addColorMatch(kBlueTarget);
        matchy.addColorMatch(kGreenTarget);
        matchy.addColorMatch(kRedTarget);
        matchy.addColorMatch(kYellowTarget);
        m_servoState = ColorServoState.down;
    }

    @Override
    public void periodic() {
        /*SmartDashboard.putNumber("red", sensy.getColor().red);
        SmartDashboard.putNumber("green", sensy.getColor().green);
        SmartDashboard.putNumber("blue", sensy.getColor().blue);
        SmartDashboard.putString("match", checkColor());*/
    }

    public Color detectedColor() {
        return sensy.getColor();
    }

    public String checkColor() {
        String colorString;
        ColorMatchResult match = matchy.matchClosestColor(detectedColor());

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }
        return colorString;
    }

    public void turnWheelClock() {
        motor.set(1);
    }

    public void turnWheelAnti() {
        motor.set(-1);
    }

    public void turnWheelSlow(int direction) {
        motor.set(0.75 * direction);
        
    }

    public void turnMotorOff() {
        motor.set(0);
    }

    public void moveServoUp() {
        servey.set(0.21);
        m_servoState = ColorServoState.up;
    }

    public void moveServoDown() {
        servey.set(0.75);
        m_servoState = ColorServoState.down;
    }

    public boolean isServoUp() {
        if (m_servoState == ColorServoState.up) {
            return true;
        }
        else {
            return false;
        }
    }
}