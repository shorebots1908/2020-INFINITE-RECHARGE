/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  private final TalonSRX _leftMaster = new TalonSRX(1);
  private final TalonSRX _rightMaster = new TalonSRX(2);
  private final Joystick _gamepad = new Joystick(0);
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private boolean isForward = false;
  Gyro gyro = new ADXRS450_Gyro();
  Servo BruhServo = new Servo(1);
  AnalogInput analog = new AnalogInput(0);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  


  // Drive PIDS
  final double STEER_K = 0.03;                    // how hard to turn toward the target
  final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
  final double DESIRED_TARGET_AREA = 3.0;         // Area of the target when the robot reaches the wall
  final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

  @Override
	public void teleopInit(){
		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMaster.configFactoryDefault();
		_rightMaster.configFactoryDefault();
		
		/* Set Neutral mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/* Configure output direction */
		_leftMaster.setInverted(true);
		_rightMaster.setInverted(false);

		gyro.reset();

		m_colorMatcher.addColorMatch(kBlueTarget);
		m_colorMatcher.addColorMatch(kGreenTarget);
		m_colorMatcher.addColorMatch(kRedTarget);
		m_colorMatcher.addColorMatch(kYellowTarget);
		
		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
	}

  @Override
	public void teleopPeriodic() {		
		/* Limelight processing */
		Update_Limelight_Tracking();
		
		/* Gamepad processing */
		double forward = _gamepad.getY();
		double turn = _gamepad.getTwist();
		double throttle = _gamepad.getThrottle();
		boolean auto = _gamepad.getRawButton(1);		
		forward = -Deadband(forward, 0.05);
		turn = Deadband(turn, 0.4);
		SmartDashboard.putNumber("Pre-Throttle", throttle);
		throttle = (throttle - 1) / 2;
		boolean servoButton = _gamepad.getRawButton(2);
		Color detectedColor = m_colorSensor.getColor();
		
		double IR = m_colorSensor.getIR();
		int proximity = m_colorSensor.getProximity();
	

		SmartDashboard.putNumber("Gryo angle", gyro.getRate());
		SmartDashboard.putNumber("Throttle", throttle);
		SmartDashboard.putNumber("Throttle Math", forward * throttle);
		SmartDashboard.putNumber("forward", forward);
		SmartDashboard.putNumber("turn", turn);
		SmartDashboard.putNumber("Analog value" ,analog.getValue());
		SmartDashboard.putNumber("Analog voltage" ,analog.getVoltage());
    	SmartDashboard.putNumber("IR", IR);
		SmartDashboard.putNumber("Proximity", proximity);
		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */

		String colorString;
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

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

		SmartDashboard.putNumber("Red", detectedColor.red);
    	SmartDashboard.putNumber("Green", detectedColor.green);
		SmartDashboard.putNumber("Blue", detectedColor.blue);
		SmartDashboard.putNumber("Confidence", match.confidence);
		SmartDashboard.putString("Detected Color", colorString);

		if (servoButton) {
			BruhServo.setAngle(75);
		}
		else {
			BruhServo.setAngle(5);
		}

		if (forward > 0 && turn == 0) {
			if(!isForward){
				gyro.reset();
				isForward = true;
			} else {
				if(gyro.getAngle() != 0){
					turn = -(gyro.getAngle() * STEER_K);
				}
			}			
		} else{
			isForward = false;
		}
		
		SmartDashboard.putNumber("turn after gyro", turn);

		if (auto)
        {
          if (m_LimelightHasValidTarget){
				_leftMaster.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.ArbitraryFeedForward, +m_LimelightSteerCommand);
				_rightMaster.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.ArbitraryFeedForward, -m_LimelightSteerCommand);
	
          } else {
				_leftMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, -0);
				_rightMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, +0);

          }
        } else  {
			_leftMaster.set(ControlMode.PercentOutput, forward * throttle, DemandType.ArbitraryFeedForward, +turn * throttle );
			_rightMaster.set(ControlMode.PercentOutput, forward * throttle, DemandType.ArbitraryFeedForward, -turn * throttle);
		}

		SmartDashboard.putBoolean("isForward", isForward);
	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value, double deadband) {

		/* Upper deadband */
		if (value >= +deadband){
			return value;
		}			
		
		/* Lower deadband */
		if (value <= -deadband){
			return value;
		}			
		
		/* Outside deadband */
		return 0;
	}



	public void Update_Limelight_Tracking()
	{
		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

		if (tv < 1.0)
		{
			m_LimelightHasValidTarget = false;
			m_LimelightDriveCommand = 0.0;
			m_LimelightSteerCommand = 0.0;
			return;
		}

		m_LimelightHasValidTarget = true;

		// Start with proportional steering
		double steer_cmd = tx * STEER_K;
		m_LimelightSteerCommand = steer_cmd;

		// try to drive forward until the target area reaches our desired area
		double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

		// don't let the robot drive too fast into the goal
		if (drive_cmd > MAX_DRIVE)
		{
			drive_cmd = MAX_DRIVE;
		}
		m_LimelightDriveCommand = drive_cmd;
	}
	
}