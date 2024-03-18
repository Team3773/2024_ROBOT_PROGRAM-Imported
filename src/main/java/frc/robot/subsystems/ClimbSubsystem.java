package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax m_LiftMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private String smartDashboardPrefix = "";

  double currentRotationSetpoint = 0;
  double armRotationStepValue = 0.5;
  double positionConverstionFactor = 0.00925;

  public ClimbSubsystem(int motorPort, boolean invertMotor, String smartDashboardPrefix) {
    m_LiftMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_LiftMotor.restoreFactoryDefaults();
    m_LiftMotor.setInverted(invertMotor);
    m_LiftMotor.setSmartCurrentLimit(60);
    this.smartDashboardPrefix = smartDashboardPrefix;
    m_pidController = m_LiftMotor.getPIDController();
    // Encoder object created to display position values
    m_encoder = m_LiftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    m_pidController.setFeedbackDevice(m_encoder);
    currentRotationSetpoint = 0;

    // m_encoder.setPositionConversionFactor(positionConverstionFactor);
    // Might be required to initialize the encoder if it is in a known location on
    // startup.

    // Brake is not needed with the PID Controller holding position
    // m_armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // Add Default values
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

  public void initDashboard() {
    SmartDashboard.putNumber(smartDashboardPrefix + "P Gain", kP);
    SmartDashboard.putNumber(smartDashboardPrefix + "I Gain", kI);
    SmartDashboard.putNumber(smartDashboardPrefix + "D Gain", kD);
    SmartDashboard.putNumber(smartDashboardPrefix + "I Zone", kIz);
    SmartDashboard.putNumber(smartDashboardPrefix + "Feed Forward", kFF);
    SmartDashboard.putNumber(smartDashboardPrefix + "Max Output", kMaxOutput);
    SmartDashboard.putNumber(smartDashboardPrefix + "Min Output", kMinOutput);
  }

  @Override
  public void periodic() {   
    m_pidController.setReference(currentRotationSetpoint, CANSparkBase.ControlType.kPosition);
    SmartDashboard.putNumber(smartDashboardPrefix + "Set Rotations", currentRotationSetpoint);
    SmartDashboard.putNumber(smartDashboardPrefix + "Encoder Position", m_encoder.getPosition());
  }

  public void incrementArmPosition() {
    currentRotationSetpoint += armRotationStepValue;
  }

  public void decrementArmPosition() {
    currentRotationSetpoint -= armRotationStepValue;
  }

  public void goToPosition(double value){
    currentRotationSetpoint = value;
  }

  // public void runLift(double speed) {
  // System.out.println(smartDashboardPrefix + "Lift speed:" + speed *
  // speedModifier);
  // m_LiftMotor.set(speed * speedModifier);
  // }

  public void stopLift() {
    var speed = 0;
    System.out.println(smartDashboardPrefix + "Lift speed:" + speed);
    m_LiftMotor.set(speed);
  }
}
