/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class Robot extends TimedRobot {
  private static final int leftID = 8;
  private static final int rightID = 7;
  private CANSparkMax left_motor;
  private CANPIDController left_pidController;
  private CANEncoder left_encoder;
  private CANSparkMax right_motor;
  private CANPIDController right_pidController;
  private CANEncoder right_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  DifferentialDrive diff_Drive;

  public Joystick joystick1 = new Joystick(0);

  @Override
  public void robotInit() {
    // initialize motor
    left_motor = new CANSparkMax(leftID, MotorType.kBrushless);
    right_motor = new CANSparkMax(rightID, MotorType.kBrushless);
    //diff_Drive = new DifferentialDrive(left_motor, right_motor);
    

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    left_motor.restoreFactoryDefaults();
    right_motor.restoreFactoryDefaults();

    left_motor.setInverted(true);
    right_motor.setInverted(false);

    

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

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
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

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }

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


  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { left_pidController.setP(p); kP = p; }
    if((i != kI)) { left_pidController.setI(i); kI = i; }
    if((d != kD)) { left_pidController.setD(d); kD = d; }
    if((iz != kIz)) { left_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { left_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      left_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { left_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { left_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { left_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { left_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    if((p != kP)) { right_pidController.setP(p); kP = p; }
    if((i != kI)) { right_pidController.setI(i); kI = i; }
    if((d != kD)) { right_pidController.setD(d); kD = d; }
    if((iz != kIz)) { right_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { right_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      right_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { right_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { right_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { right_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { right_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint,setPointTurn, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      //setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      setPoint = Deadband(joystick1.getY() * -maxV, 100);
      setPointTurn = Deadband(joystick1.getTwist() * -maxV, 190);
      SmartDashboard.putNumber("Joystick", setPoint);
      SmartDashboard.putNumber("Joystick Turn", setPointTurn);
      left_pidController.setReference(setPoint + setPointTurn, ControlType.kVelocity);
      right_pidController.setReference(setPoint - setPointTurn, ControlType.kVelocity);
      processVariable = left_encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      left_pidController.setReference(setPoint, ControlType.kSmartMotion);
      right_pidController.setReference(setPoint, ControlType.kSmartMotion);
      processVariable = left_encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", left_motor.getAppliedOutput());
  }
}
