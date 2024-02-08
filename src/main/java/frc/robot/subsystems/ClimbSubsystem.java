package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax m_liftMotor;
    
  public ClimbSubsystem(CANSparkMax m_liftMotor) {
    this.m_liftMotor = m_liftMotor;
  }

  public ClimbSubsystem() {

  m_liftMotor = new CANSparkMax (RobotMap.m_liftMotorPort,MotorType.kBrushless);
    m_liftMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runLift(double speed) {
    System.out.println("Lift speed:" + speed);
    m_liftMotor.set(speed);
  }

  public void stopLift() {
    var speed = 0;
    System.out.println("Lift speed:" + speed);
    m_liftMotor.set(speed);
  }

}
