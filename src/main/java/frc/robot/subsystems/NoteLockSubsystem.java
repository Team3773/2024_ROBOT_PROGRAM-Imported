// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class NoteLockSubsystem extends SubsystemBase {

  private CANSparkMax m_lockMotor;
  private static final double speedModifier = 1.0;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private String smartDashboardPrefix = "";
  private SparkPIDController m_pidController;

  double currentRotationSetpoint = 0;
  double armRotationStepValue = 1.0;

  public NoteLockSubsystem(int motorPort, boolean invertMotor, String smartDashboardPrefix) {
    this.smartDashboardPrefix = smartDashboardPrefix;
    m_lockMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_lockMotor.restoreFactoryDefaults();
    m_lockMotor.setInverted(invertMotor);
    m_lockMotor.setSmartCurrentLimit(40);
    m_lockMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pidController = m_lockMotor.getPIDController();    
    m_encoder = m_lockMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    // m_encoder.setPosition(0);
    m_pidController.setFeedbackDevice(m_encoder);
    kP = 0.1;
    kI = 1e-5;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    m_pidController.setReference(currentRotationSetpoint, CANSparkBase.ControlType.kPosition);
    SmartDashboard.putNumber(smartDashboardPrefix + "Set Rotations", currentRotationSetpoint);
    SmartDashboard.putNumber(smartDashboardPrefix + "Encoder Position", m_encoder.getPosition());
  }

  public void incrementPosition() {
    currentRotationSetpoint += armRotationStepValue;
  }

  public void decrementPosition() {
    currentRotationSetpoint -= armRotationStepValue;
  }

  public void goToPosition(double value){
    currentRotationSetpoint = value;
  }

  // public void runLift(double speed) {
  //   System.out.println("Lock speed:" + speed * speedModifier);
  //   m_lockMotor.set(speed * speedModifier);
  // }

  public void stopLift() {
    var speed = 0;
    System.out.println("Lock speed:" + speed);
    m_lockMotor.set(speed);
  }
}
