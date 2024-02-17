package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax m_leftLiftMotor;
  private CANSparkMax m_rightLiftMotor;
    
  public ClimbSubsystem(CANSparkMax m_leftLiftMotor, CANSparkMax m_rightLiftMotor) {
    this.m_leftLiftMotor = m_leftLiftMotor;
    this.m_rightLiftMotor = m_rightLiftMotor;
  }

  public ClimbSubsystem() {

  m_leftLiftMotor = new CANSparkMax (RobotMap.m_leftLiftMotorPort,MotorType.kBrushless);
    m_leftLiftMotor.setInverted(false);
  m_rightLiftMotor = new CANSparkMax (RobotMap.m_rightLiftMotorPort,MotorType.kBrushless);
    m_rightLiftMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runLift(double speed) {
    System.out.println("Lift speed:" + speed);
    m_leftLiftMotor.set(speed);
    m_rightLiftMotor.set(speed);
  }

  public void stopLift() {
    var speed = 0;
    System.out.println("Lift speed:" + speed);
    m_leftLiftMotor.set(speed);
    m_rightLiftMotor.set(speed);
  }

}
