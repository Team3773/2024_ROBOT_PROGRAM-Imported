package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax m_leftLiftMotor;
  private CANSparkMax m_rightLiftMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private static final double speedModifier = .5;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  double currentRotationSetpoint = 0;
  double armRotationStepValue = 0.01;
  // com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_armMotor = new
  // WPI_TalonSRX(RobotMap.m_armMotorPort); change motor contgroller to SparkMax
  // for neo 1.1 motor

  public ClimbSubsystem() {
    m_leftLiftMotor = new CANSparkMax(RobotMap.m_leftLiftMotorPort, MotorType.kBrushless);
    m_leftLiftMotor.restoreFactoryDefaults();
    m_leftLiftMotor.setInverted(false);
    m_rightLiftMotor = new CANSparkMax(RobotMap.m_rightLiftMotorPort, MotorType.kBrushless);
    m_rightLiftMotor.restoreFactoryDefaults();
    m_rightLiftMotor.setInverted(true);

    m_pidController = m_leftLiftMotor.getPIDController();
    m_pidController = m_rightLiftMotor.getPIDController();
    // Encoder object created to display position values
    m_encoder = m_leftLiftMotor.getEncoder();
    m_encoder = m_rightLiftMotor.getEncoder();  
    //Might be required to initialize the encoder if it is in a known location on startup.
    //m_encoder.setPosition(0);
    // Brake is not needed with the PID Controller holding position
    //m_armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //Add Default values
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

  public void initDashboard(){
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", currentRotationSetpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    currentRotationSetpoint = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    m_pidController.setReference(currentRotationSetpoint, CANSparkMax.ControlType.kPosition);
    
    
    SmartDashboard.putNumber("Note Arm setPoint", currentRotationSetpoint);
    SmartDashboard.putNumber("Note Arm Encoder Position", m_encoder.getPosition());
  }

  public void incrementArmPosition(){
    currentRotationSetpoint += armRotationStepValue;
  }
  public void decrementArmPosition(){
    currentRotationSetpoint -= armRotationStepValue;
  }

  public void runLift(double speed) {
    System.out.println("Intake speed:" + speed * speedModifier);    
    m_leftLiftMotor.set(speed * speedModifier);
    m_rightLiftMotor.set(speed * speedModifier);
  }

  public void stopLift() {
    var speed = 0;
    System.out.println("Intake speed:" + speed);
    m_leftLiftMotor.set(speed);
    m_rightLiftMotor.set(speed);
  }
}
