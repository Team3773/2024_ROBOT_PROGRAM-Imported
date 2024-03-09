// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class NoteLockSubsystem extends SubsystemBase {

  private CANSparkMax m_lockMotor;
  private static final double speedModifier = 1.0;
  
  public NoteLockSubsystem() {
    m_lockMotor = new CANSparkMax (RobotMap.m_lockMotorPort,MotorType.kBrushed);
  }

  public void runLift(double speed) {
    System.out.println("Intake speed:" + speed * speedModifier);    
    m_lockMotor.set(speed * speedModifier);
  }

  public void stopLift() {
    var speed = 0;
    System.out.println("Intake speed:" + speed);
    m_lockMotor.set(speed);
  }
}
