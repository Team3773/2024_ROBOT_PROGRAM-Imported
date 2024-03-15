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
  double armRotationStepValue = 0.1;

  public NoteLockSubsystem(int motorPort, boolean invertMotor, String smartDashboardPrefix) {
    this.smartDashboardPrefix = smartDashboardPrefix;
    m_lockMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_lockMotor.restoreFactoryDefaults();
    m_lockMotor.setInverted(invertMotor);
    m_lockMotor.setSmartCurrentLimit(5);
    m_lockMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pidController = m_lockMotor.getPIDController();    
    m_encoder = m_lockMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    // m_encoder.setPosition(0);
    m_pidController.setFeedbackDevice(m_encoder);
    kP = 0.1;
    kI = 1e-4;
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

  

    
    // public void initDashboard() {
    // SmartDashboard.putNumber(smartDashboardPrefix + "P Gain", kP);
    // SmartDashboard.putNumber(smartDashboardPrefix + "I Gain", kI);
    // SmartDashboard.putNumber(smartDashboardPrefix + "D Gain", kD);
    // SmartDashboard.putNumber(smartDashboardPrefix + "I Zone", kIz);
    // SmartDashboard.putNumber(smartDashboardPrefix + "Feed Forward", kFF);
    // SmartDashboard.putNumber(smartDashboardPrefix + "Max Output", kMaxOutput);
    // SmartDashboard.putNumber(smartDashboardPrefix + "Min Output", kMinOutput);
    // }
    // public void periodic() {
    // This method will be called once per scheduler run
    
    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber(smartDashboardPrefix + "P Gain", 0);
    // double i = SmartDashboard.getNumber(smartDashboardPrefix + "I Gain", 0);
    // double d = SmartDashboard.getNumber(smartDashboardPrefix + "D Gain", 0);
    // double iz = SmartDashboard.getNumber(smartDashboardPrefix + "I Zone", 0);
    // double ff = SmartDashboard.getNumber(smartDashboardPrefix + "Feed Forward",
    // 0);
    // double max = SmartDashboard.getNumber(smartDashboardPrefix + "Max Output",
    // 0);
    // double min = SmartDashboard.getNumber(smartDashboardPrefix + "Min Output",
    // 0);
    // currentRotationSetpoint = SmartDashboard.getNumber(smartDashboardPrefix +
    // "Set Rotations", 0);
    
    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if((p != kP)) { m_pidController.setP(p); kP = p; }
    // if((i != kI)) { m_pidController.setI(i); kI = i; }
    // if((d != kD)) { m_pidController.setD(d); kD = d; }
    // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) {
    // m_pidController.setOutputRange(min, max);
    // kMinOutput = min; kMaxOutput = max;
    // }
  /* */
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
