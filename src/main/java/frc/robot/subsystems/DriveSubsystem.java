// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {

  //private static final double kMaxSpeed = 3.0; // meters per second
  //private static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation
  private WPI_TalonSRX m_leftMotor;
  private WPI_TalonSRX m_rightMotor;
  private WPI_TalonSRX m_leftMotorFollower;
  private WPI_TalonSRX m_rightMotorFollower;
  private DifferentialDrive drive;
  
  /* private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;
   Creates a new DriveTrain. */
  public DriveSubsystem() {
    m_leftMotor = new WPI_TalonSRX(RobotMap.m_leftMotorPort);
    m_rightMotor = new WPI_TalonSRX(RobotMap.m_rightMotorPort);
    m_leftMotorFollower = new WPI_TalonSRX(RobotMap.m_leftMotorFollowerPort);
    m_rightMotorFollower = new WPI_TalonSRX(RobotMap.m_rightMotorFollowerPort);
    m_leftMotorFollower.follow(m_leftMotor);
    m_rightMotorFollower.follow(m_rightMotor);
    drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftMotor.setSafetyEnabled(true);
    m_leftMotor.setSafetyEnabled(false);
    m_leftMotor.setExpiration(.1);
    m_leftMotor.feed();
    m_rightMotor.setSafetyEnabled(true);
    m_rightMotor.setSafetyEnabled(false);
    m_rightMotor.setExpiration(.1);
    m_rightMotor.feed();
  }
  
  public void arcadeDrive(double speed, double rotation) {
    double speedModifier = .7;
    double rotationModifier = .5;
    drive.arcadeDrive(speed * speedModifier, rotation * rotationModifier); 
    
  }
}
