package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax m_armMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private static final double speedModifier = 0.7;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  double currentRotationSetpoint = 0;
  double armRotationStepValue = 0.1;

  // com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_armMotor = new
  // WPI_TalonSRX(RobotMap.m_armMotorPort); change motor contgroller to SparkMax
  // for neo 1.1 motor

  public IntakeSubsystem() {
    m_armMotor = new CANSparkMax(RobotMap.m_armMotorPort, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setInverted(false);
    m_armMotor.setSmartCurrentLimit(60);
    m_pidController = m_armMotor.getPIDController();
    // Encoder object created to display position values
    m_encoder = m_armMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);  
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

  @Override
  public void periodic() {
    m_pidController.setReference(currentRotationSetpoint, CANSparkMax.ControlType.kPosition);       
    SmartDashboard.putNumber("Note Arm setPoint", currentRotationSetpoint);
    SmartDashboard.putNumber("Note Arm Encoder Position", m_encoder.getPosition());
  }

  public void incrementPosition(){
    currentRotationSetpoint += armRotationStepValue;
  }
  public void decrementPosition(){
    currentRotationSetpoint -= armRotationStepValue;
  }

  public void goToPosition(double value){
    currentRotationSetpoint = value;
  }

  // public void runLift(double speed) {
  //   System.out.println("Intake speed:" + speed * speedModifier);    
  //   m_armMotor.set(speed * speedModifier);
  // }

  public void stopLift() {
    var speed = 0;
    System.out.println("Intake speed:" + speed);
    m_armMotor.set(speed);
  }

}
