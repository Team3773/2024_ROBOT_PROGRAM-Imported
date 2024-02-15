package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax m_armMotor;
  private static final double speedModifier = .5;
  // com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_armMotor = new
  // WPI_TalonSRX(RobotMap.m_armMotorPort); change motor contgroller to SparkMax
  // for neo 1.1 motor

  public IntakeSubsystem() {
    m_armMotor = new CANSparkMax(RobotMap.m_armMotorPort, MotorType.kBrushless);
    m_armMotor.setInverted(false);
    // Set the idle mode to brake
    m_armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // The motor will brake and resist any external motion
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    System.out.println("Intake speed:" + speed * speedModifier);    
    m_armMotor.set(speed * speedModifier);
  }

  public void stopIntake() {
    var speed = 0;
    System.out.println("Intake speed:" + speed);
    m_armMotor.set(speed);
  }

}
