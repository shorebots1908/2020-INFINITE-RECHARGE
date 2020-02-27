/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private static final int leftID = 8;
  private static final int rightID = 7;
  private CANSparkMax left_motor, right_motor, Intake, Color_Spinner;
  private CANPIDController left_pidController, right_pidController;
  private CANEncoder left_encoder, right_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  DifferentialDrive diff_Drive;
  private double startTime;

  public Joystick joystick1 = new Joystick(0);
  public Joystick joystick2 = new Joystick(1);

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private boolean isForward = false;
  //private boolean isTurn = false;
  Gyro gyro = new ADXRS450_Gyro();
  Servo BruhServo = new Servo(0);
  AnalogInput intakeSensor = new AnalogInput(0);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  public Spark ledStrip = new Spark(1);

  private final TalonSRX Shooter = new TalonSRX(5);
  private final TalonSRX Elevator_Top = new TalonSRX(6);
  private final TalonSRX Elevator_Butt = new TalonSRX(4);
  private final TalonSRX Climb = new TalonSRX(1);

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // Limelight PIDs
  final double STEER_K = 0.05; // how hard to turn toward the target
  final double DRIVE_K = 0.30; // how hard to drive fwd toward the target
  final double DESIRED_TARGET_AREA = 3.0; // Area of the target when the robot reaches the wall
  final double MAX_DRIVE = 0.5; // Simple speed limit so we don't drive too fast

  @Override
  public void robotInit() {
    // initialize motor
    left_motor = new CANSparkMax(leftID, MotorType.kBrushless);
    right_motor = new CANSparkMax(rightID, MotorType.kBrushless);
    Intake = new CANSparkMax(2, MotorType.kBrushless);
    Color_Spinner = new CANSparkMax(9, MotorType.kBrushless);

    // diff_Drive = new DifferentialDrive(left_motor, right_motor);

    left_motor.restoreFactoryDefaults();
    right_motor.restoreFactoryDefaults();

    left_motor.setInverted(true);
    right_motor.setInverted(false);

    pidInit();

    Climb.setNeutralMode(NeutralMode.Brake);
    Elevator_Butt.follow(Elevator_Top);
    Elevator_Butt.setInverted(false);

    gyro.reset();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

  }
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp(); // get the match start time
    ledStrip.set(-0.77);   
  }

  @Override
  public void autonomousPeriodic() {
    double timePassed = Timer.getFPGATimestamp() - startTime;
    System.out.println(timePassed);


    
    // kick up the climber so the intake falls down, then pull back the climber arms
    if (timePassed <= 2) {
      Climb.set(ControlMode.PercentOutput, -0.2, DemandType.ArbitraryFeedForward, 0);
    } else if ((timePassed > 2) && (timePassed < 4)) {
      Climb.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, 0);
    } else {
      Climb.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    }
    // start the elevator and shoot for 10 seconds
    if (timePassed < 10) {
        Elevator_Top.set(ControlMode.PercentOutput, 1, DemandType.ArbitraryFeedForward, 0);
        Shooter.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
    } else {
        Elevator_Top.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
        Shooter.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    }
   // drive forward for 2 seconds
    if ((timePassed > 10) && (timePassed < 12)) {
        left_motor.set(0.3);
        right_motor.set(0.3);
    } else {
        left_motor.set(0);
        right_motor.set(0);
    }  
  }

  @Override
  public void teleopPeriodic() {

    Update_Limelight_Tracking();

    double setPoint, setPointTurn, leftVelocity, rightVelocity;
    /* Gamepad processing */
    double forward = joystick1.getY();
    double turn = joystick1.getTwist();
    double throttle = joystick1.getThrottle();
    boolean auto = joystick1.getRawButton(1);
    forward = Deadband(forward, 0.05);
    turn = Deadband(turn, 0.4);
    SmartDashboard.putNumber("Pre-Throttle", throttle);
    throttle = (throttle - 1) / 2;
    boolean servoButton = joystick1.getRawButton(2);
    boolean driveToWheel = joystick2.getRawButton(1);

    int POV = joystick2.getPOV();

    boolean climbUp = joystick1.getRawButton(8);
    boolean climbDown = joystick1.getRawButton(10);

    boolean intakeButton = joystick1.getRawButton(3);
    boolean bruhtakeButton = joystick1.getRawButton(5); 


    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();
    getColor();

    /// BUTTONS START///
    if (servoButton) {
      BruhServo.setAngle(90);
    } else {
      BruhServo.setAngle(20);
    }

    if (driveToWheel) {
      if (proximity < 100) {
        forward = -0.3;
      } else {forward = 0;}
    }

    if (climbUp) {
      Climb.set(ControlMode.PercentOutput, 1, DemandType.ArbitraryFeedForward, 0);
      ledStrip.set(0.57);
    } else if (climbDown) {
      Climb.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
      ledStrip.set(0.57);
    } else {
      Climb.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    }

    if (POV == 0) {
      Elevator_Top.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
    }
    if (POV == 180) {
      Elevator_Top.set(ControlMode.PercentOutput, 1, DemandType.ArbitraryFeedForward, 0);
    }
    if (POV == 270){
      Color_Spinner.set(0.5);
    }
    if(POV == 90){
      Color_Spinner.set(-0.5);
    }
    if (POV == -1) {
      Elevator_Top.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
      Color_Spinner.set(0);
    }

    if (intakeButton) {
      Intake.set(0.5);
     } else if {
     (intakeSensor.getValue() >= 600); 
     } 
     else {
      Intake.set(0);
    }


    /// BUTTONS END ///

    /// DRIVE START ///
    // setPoint = SmartDashboard.getNumber("Set Velocity", 0);

    if (forward != 0 && turn == 0) {
      if (!isForward) {
        gyro.reset();
        isForward = true;
      } else {
        if (gyro.getAngle() != 0) {
          turn = -(gyro.getAngle() * STEER_K);
        }
      }
    } else {
      isForward = false;
    }

    if (auto) {
      Shooter.set(ControlMode.PercentOutput, -1, DemandType.ArbitraryFeedForward, 0);
    } else {
      // diffDrive.arcadeDrive(forward * throttle, -turn * throttle);
      Shooter.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    }




    /* if (auto) {
      if (m_LimelightHasValidTarget) {
        forward = m_LimelightDriveCommand;
        turn = m_LimelightSteerCommand;
        // ledStrip.set(-0.09);
      } else {
        forward = 0;
        turn = 0;
        // ledStrip.set(0.87);
      }
    } else {
      // diffDrive.arcadeDrive(forward * throttle, -turn * throttle);
      Shooter.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    } */

    setPoint = Deadband(forward * -maxVel, 100);
    setPointTurn = Deadband(turn * -maxVel, 190);
    SmartDashboard.putNumber("Joystick", setPoint);
    SmartDashboard.putNumber("Joystick Turn", setPointTurn);
    left_pidController.setReference(setPoint + setPointTurn, ControlType.kVelocity);
    right_pidController.setReference(setPoint - setPointTurn, ControlType.kVelocity);
    leftVelocity = left_encoder.getVelocity();
    rightVelocity = right_encoder.getVelocity();

    /// DRIVE END ///

    /// SMART DASHBOARD UPDATE ///
    SmartDashboard.putBoolean("isForward", isForward);
    SmartDashboard.putNumber("Gryo angle", gyro.getRate());
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Throttle Math", forward * throttle);
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("turn", turn);
    SmartDashboard.putNumber("Analog value", analog.getValue());
    SmartDashboard.putNumber("Analog voltage", analog.getVoltage());
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("POV", POV);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Left Vel", leftVelocity);
    SmartDashboard.putNumber("Right Vel", rightVelocity);
    SmartDashboard.putNumber("Output", left_motor.getAppliedOutput());

  }

  double Deadband(double value, double deadband) {
    /* Upper deadband */
    if (value >= +deadband) {
      return value;
    }
    /* Lower deadband */
    if (value <= -deadband) {
      return value;
    }
    /* Outside deadband */
    return 0;
  }

  public void Update_Limelight_Tracking() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0) {
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
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }

  public void pidInit() {
    // initialze PID controller and encoder objects
    left_pidController = left_motor.getPIDController();
    left_encoder = left_motor.getEncoder();
    right_pidController = right_motor.getPIDController();
    right_encoder = right_motor.getEncoder();

    // PID coefficients
    kP = 0.00005;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000156;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    left_pidController.setP(kP);
    left_pidController.setI(kI);
    left_pidController.setD(kD);
    left_pidController.setIZone(kIz);
    left_pidController.setFF(kFF);
    left_pidController.setOutputRange(kMinOutput, kMaxOutput);

    right_pidController.setP(kP);
    right_pidController.setI(kI);
    right_pidController.setD(kD);
    right_pidController.setIZone(kIz);
    right_pidController.setFF(kFF);
    right_pidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    left_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    left_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    left_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    left_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    right_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    right_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    right_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    right_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);
  }

  public void getColor() {
    String colorString;
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      ledStrip.set(0.83);
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      ledStrip.set(0.61);
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      ledStrip.set(0.77);
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      ledStrip.set(0.69);
    } else {
      colorString = "Unknown";
      ledStrip.set(0.03);
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}
