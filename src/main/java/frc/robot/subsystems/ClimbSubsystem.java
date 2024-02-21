package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax m_LiftMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private static final double speedModifier = .5;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private String smartDashboardPrefix = "";

  double currentRotationSetpoint = 0;
  double armRotationStepValue = 0.01;
  // com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_armMotor = new
  // WPI_TalonSRX(RobotMap.m_armMotorPort); change motor contgroller to SparkMax
  // for neo 1.1 motor

  public ClimbSubsystem(int motorPort, boolean invertMotor, String smartDashboardPrefix){
    m_LiftMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    m_LiftMotor.restoreFactoryDefaults();
    m_LiftMotor.setInverted(invertMotor);

    this.smartDashboardPrefix = smartDashboardPrefix;

    m_pidController = m_LiftMotor.getPIDController();
    // Encoder object created to display position values
    m_encoder = m_LiftMotor.getEncoder();
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
    SmartDashboard.putNumber(smartDashboardPrefix + "P Gain", kP);
    SmartDashboard.putNumber(smartDashboardPrefix + "I Gain", kI);
    SmartDashboard.putNumber(smartDashboardPrefix + "D Gain", kD);
    SmartDashboard.putNumber(smartDashboardPrefix + "I Zone", kIz);
    SmartDashboard.putNumber(smartDashboardPrefix + "Feed Forward", kFF);
    SmartDashboard.putNumber(smartDashboardPrefix + "Max Output", kMaxOutput);
    SmartDashboard.putNumber(smartDashboardPrefix + "Min Output", kMinOutput);
    SmartDashboard.putNumber(smartDashboardPrefix + "Set Rotations", currentRotationSetpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber(smartDashboardPrefix + "P Gain", 0);
    double i = SmartDashboard.getNumber(smartDashboardPrefix + "I Gain", 0);
    double d = SmartDashboard.getNumber(smartDashboardPrefix + "D Gain", 0);
    double iz = SmartDashboard.getNumber(smartDashboardPrefix + "I Zone", 0);
    double ff = SmartDashboard.getNumber(smartDashboardPrefix + "Feed Forward", 0);
    double max = SmartDashboard.getNumber(smartDashboardPrefix + "Max Output", 0);
    double min = SmartDashboard.getNumber(smartDashboardPrefix + "Min Output", 0);
    currentRotationSetpoint = SmartDashboard.getNumber(smartDashboardPrefix + "Set Rotations", 0);

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
    
    
    SmartDashboard.putNumber(smartDashboardPrefix + "Note Arm setPoint", currentRotationSetpoint);
    SmartDashboard.putNumber(smartDashboardPrefix + "Note Arm Encoder Position", m_encoder.getPosition());
  }

  public void incrementArmPosition(){
    currentRotationSetpoint += armRotationStepValue;
  }
  public void decrementArmPosition(){
    currentRotationSetpoint -= armRotationStepValue;
  }

  public void runLift(double speed) {
    System.out.println(smartDashboardPrefix + "Intake speed:" + speed * speedModifier);    
    m_LiftMotor.set(speed * speedModifier);
  }

  public void stopLift() {
    var speed = 0;
    System.out.println(smartDashboardPrefix + "Intake speed:" + speed);
    m_LiftMotor.set(speed);
  }
}
