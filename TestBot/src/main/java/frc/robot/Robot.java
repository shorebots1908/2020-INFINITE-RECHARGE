/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
/* import edu.wpi.first.wpilibj.drive.DifferentialDrive; */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/* import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand; */
import edu.wpi.first.networktables.*;


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
  Gyro gyro = new ADXRS450_Gyro(); /* SPI.Port.kMXP */

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
		_leftMaster.setInverted(false);
		_rightMaster.setInverted(true);

		gyro.reset();
		
		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
	}

  @Override
	public void teleopPeriodic() {		
		/* Limelight processing */
		Update_Limelight_Tracking();
		SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
		
		/* Gamepad processing */
		double forward = _gamepad.getY();
		double turn = _gamepad.getTwist();
		double throttle = _gamepad.getThrottle();
		boolean auto = _gamepad.getRawButton(1);		
		forward = Deadband(forward);
		turn = Deadband(turn);
		throttle = (throttle - 1) / 2;

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */

	
		if (auto)
        {
          if (m_LimelightHasValidTarget)
          {
				_leftMaster.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.ArbitraryFeedForward, +m_LimelightSteerCommand);
				_rightMaster.set(ControlMode.PercentOutput, m_LimelightDriveCommand, DemandType.ArbitraryFeedForward, -m_LimelightSteerCommand);
	
          }
          else
          {
				_leftMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, -0);
				_rightMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, +0);

          }
        }
        else
        {
			_leftMaster.set(ControlMode.PercentOutput, forward * throttle, DemandType.ArbitraryFeedForward, -turn * throttle);
			_rightMaster.set(ControlMode.PercentOutput, forward * throttle, DemandType.ArbitraryFeedForward, +turn * throttle);
		
		}
	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}



	public void Update_Limelight_Tracking()
	{
		// These numbers must be tuned for your Robot!  Be careful!
		final double STEER_K = 0.03;                    // how hard to turn toward the target
		final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
		final double DESIRED_TARGET_AREA = 3.0;        // Area of the target when the robot reaches the wall
		final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

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