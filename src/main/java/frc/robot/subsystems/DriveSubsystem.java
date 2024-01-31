// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import javax.management.loading.PrivateClassLoader;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


public class DriveSubsystem<TalonSRX> extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation
  //per second

  
  //private static final double kTrackWidth = 0.381 * 2; // meters
  //private static final double kWheelRadius = 0.0508; // meters
  //private static final int kEncoderResolution = 4096;
  /** Creates a new DriveTrain. */
  
  private DriveSubsystem() {
    final Talon m_leftMotor = new Talon(RobotMap.m_leftMotorPort);
    final Talon m_rightMotor = new Talon(RobotMap.m_rightMotorPort);
    final Talon m_leftMotorFollower = new Talon(RobotMap.m_leftMotorFollowerPort);
    final Talon m_rightMotorFollower = new Talon(RobotMap.m_rightMotorFollowerPort);
    
  final DifferentialDrive drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }
}
