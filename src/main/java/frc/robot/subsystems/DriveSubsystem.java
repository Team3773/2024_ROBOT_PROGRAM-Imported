// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import javax.management.loading.PrivateClassLoader;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;


public class DriveSubsystem<TalonSRX> extends SubsystemBase {

  // public static final double kMaxSpeed = 3.0; // meters per second
  // public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation
  // per second

  // private static final double kTrackWidth = 0.381 * 2; // meters
  // private static final double kWheelRadius = 0.0508; // meters
  // private static final int kEncoderResolution = 4096;
  /** Creates a new DriveTrain. */
  private TalonSRX m_leftMotor;
  private TalonSRX m_leftMotorFollower;
  private TalonSRX m_rightMotor;
  private TalonSRX m_rightMotorFollower;

  private DifferentialDrive drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  public DriveSubsystem() {
    com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(RobotMap.m_leftMotorFollowerPort);
    com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(RobotMap.m_rightMotorPort);
    com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_leftMotorFollower = new WPI_TalonSRX(RobotMap.m_leftMotorFollowerPort);
    com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_rightMotorFollower = new WPI_TalonSRX(RobotMap.m_rightMotorFollowerPort);
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }
}
