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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax _leftMaster = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax _rightMaster = new CANSparkMax(8, MotorType.kBrushless);
  DifferentialDrive _diffDrive = new DifferentialDrive(_leftMaster, _rightMaster);
  private CANPIDController leftPid,rightPid;
  private CANEncoder encoderLeft,encoderRight;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private final Joystick _gamepad = new Joystick(0);
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private boolean isForward = false;
  private boolean isTurn = false;
  Gyro gyro = new ADXRS450_Gyro();
  Servo BruhServo = new Servo(0);
  AnalogInput analog = new AnalogInput(0);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  //public Spark ledStrip = new Spark(1);
  
  private final TalonSRX Shooter = new TalonSRX(6);
  private final TalonSRX Elevator_Top = new TalonSRX(5);
  private final TalonSRX Elevator_Butt = new TalonSRX(4);
  private final TalonSRX Climb = new TalonSRX(1);
  private final TalonSRX Intake = new TalonSRX(2);
  private final TalonSRX Color_Spinner = new TalonSRX(9);


  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  


  // Drive PIDS
  final double STEER_K = 0.05;                    // how hard to turn toward the target
  final double DRIVE_K = 0.30;                    // how hard to drive fwd toward the target
  final double DESIRED_TARGET_AREA = 3.0;         // Area of the target when the robot reaches the wall
  final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

  @Override
	public void teleopInit(){
		/* Ensure motor output is neutral during init */
		//_leftMaster.set(ControlMode.PercentOutput, 0);
		///_rightMaster.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		//_leftMaster.configFactoryDefault();
		//_rightMaster.configFactoryDefault();
		
		/* Set Neutral mode */
		//_leftMaster.setNeutralMode(NeutralMode.Brake);
		//_rightMaster.setNeutralMode(NeutralMode.Brake);
		_leftMaster.setIdleMode(IdleMode.kCoast);
		_rightMaster.setIdleMode(IdleMode.kCoast);
		Climb.setNeutralMode(NeutralMode.Brake);

		/* Configure output direction */
		_leftMaster.setInverted(false);
		_rightMaster.setInverted(false);

		//ledStrip.set(0);

		Elevator_Butt.follow(Elevator_Top);
		Elevator_Butt.setInverted(true);

		leftPid = _leftMaster.getPIDController();
		rightPid = _rightMaster.getPIDController();
		encoderLeft = _leftMaster.getEncoder();
		encoderRight = _rightMaster.getEncoder();
		kP = 5e-5; 
		kI = 1e-6;
		kD = 0; 
		kIz = 0; 
		kFF = 0.000156; 
		kMaxOutput = 1; 
		kMinOutput = -1;
		maxRPM = 5700;

		rightPid.setP(kP);
		rightPid.setI(kI);
		rightPid.setD(kD);
		rightPid.setIZone(kIz);
		rightPid.setFF(kFF);
		rightPid.setOutputRange(kMinOutput, kMaxOutput);

		SmartDashboard.putBoolean("Mode", true);


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
		forward = Deadband(forward, 0.05);
		turn = Deadband(turn, 0.4);
		SmartDashboard.putNumber("Pre-Throttle", throttle);
		throttle = (throttle - 1) / 2;
		//boolean servoButtonDown = _gamepad.getRawButton(9);
		boolean servoButton = _gamepad.getRawButton(2);
		Color detectedColor = m_colorSensor.getColor();
		int POV = _gamepad.getPOV();
		boolean intakeButton = _gamepad.getRawButton(2);
		//boolean outakeButton = _gamepad.getRawButton(3);
		
		boolean climbUp = _gamepad.getRawButton(8);
		boolean climbDown = _gamepad.getRawButton(10);

		double IR = m_colorSensor.getIR();
		int proximity = m_colorSensor.getProximity();

	    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      rightPid.setReference(setPoint, ControlType.kVelocity);
      processVariable = encoderRight.getVelocity();
    } else {
	  setPoint = SmartDashboard.getNumber("Set Position", 0);
	  rightPid.setReference(setPoint, ControlType.kSmartMotion);
	  processVariable = encoderRight.getPosition();
	}
	  

		SmartDashboard.putNumber("Gryo angle", gyro.getRate());
		SmartDashboard.putNumber("Throttle", throttle);
		SmartDashboard.putNumber("Throttle Math", forward * throttle);
		SmartDashboard.putNumber("forward", forward);
		SmartDashboard.putNumber("turn", turn);
		SmartDashboard.putNumber("Analog value" ,analog.getValue());
		SmartDashboard.putNumber("Analog voltage" ,analog.getVoltage());
    	SmartDashboard.putNumber("IR", IR);
		SmartDashboard.putNumber("Proximity", proximity);
		SmartDashboard.putNumber("POV", POV);
		SmartDashboard.putNumber("Left Encoder", encoderLeft.getPosition());
		SmartDashboard.putNumber("Right PID", encoderRight.getPosition());
		SmartDashboard.putNumber("SetPoint", setPoint);
		SmartDashboard.putNumber("Process Variable", processVariable);
		SmartDashboard.putNumber("Output", _rightMaster.getAppliedOutput());

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */

		String colorString;
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

		if (match.color == kBlueTarget) {
		colorString = "Blue";
		//ledStrip.set(0.83);
		} else if (match.color == kRedTarget) {
		colorString = "Red";
		//ledStrip.set(0.61);
		} else if (match.color == kGreenTarget) {
		colorString = "Green";
		//ledStrip.set(0.77);
		} else if (match.color == kYellowTarget) {
		colorString = "Yellow";
		//ledStrip.set(0.69);
		} else {
		colorString = "Unknown";
		//ledStrip.set(0.83);
		}

		SmartDashboard.putNumber("Red", detectedColor.red);
    	SmartDashboard.putNumber("Green", detectedColor.green);
		SmartDashboard.putNumber("Blue", detectedColor.blue);
		SmartDashboard.putNumber("Confidence", match.confidence);
		SmartDashboard.putString("Detected Color", colorString);

		if (servoButton) {			
			BruhServo.setAngle(90);		
		}
		else {			
			BruhServo.setAngle(20);			
		}

		

		if(climbUp){
			Climb.set(ControlMode.PercentOutput, 1, DemandType.ArbitraryFeedForward, 0);
		}
		else if(climbDown){
			Climb.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
		} else {
			Climb.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
		}
		
		if (POV == 0) {
			Elevator_Top.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
		}
		if (POV == 180) {
			Elevator_Top.set(ControlMode.PercentOutput, 1, DemandType.ArbitraryFeedForward, 0);
		}
		if (POV == -1) {
			Elevator_Top.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
		}

		/* if (intakeButton) {
			Intake.set(ControlMode.PercentOutput, 0.5, DemandType.ArbitraryFeedForward, 0);
		} else if (outakeButton) {
			Intake.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
		} else {
			Intake.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
		} */
		//SmartDashboard.putBoolean("ServoButton", servoButton);

		if (forward != 0 && turn == 0) {
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

		/* if (auto)
        {
			Shooter.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);

		} else {
			Shooter.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
		} */
		
		
		//SmartDashboard.putNumber("turn after gyro", turn);

	 	if (auto)
        {
			//Shooter.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
          if (m_LimelightHasValidTarget){
				//_leftMaster.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.ArbitraryFeedForward, +m_LimelightSteerCommand);
				//_rightMaster.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.ArbitraryFeedForward, -m_LimelightSteerCommand);
				_diffDrive.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
				//ledStrip.set(-0.09);

          } else {
				//_leftMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, -0);
				//_rightMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, +0);
				_diffDrive.arcadeDrive(0,0);
				//ledStrip.set(0.87);
          }
        } else  {
			//_leftMaster.set(ControlMode.PercentOutput, forward * throttle, DemandType.ArbitraryFeedForward, +turn * throttle );
			//_rightMaster.set(ControlMode.PercentOutput, forward * throttle, DemandType.ArbitraryFeedForward, -turn * throttle);
			_diffDrive.arcadeDrive(forward * throttle, -turn * throttle);

			Shooter.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);

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