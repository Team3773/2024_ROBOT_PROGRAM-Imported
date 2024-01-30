package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_armMotor = new WPI_TalonSRX(RobotMap.m_armMotorPort);
  
  
  public IntakeSubsystem() {
    m_armMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    System.out.println("Intake speed:" + speed);
    m_armMotor.set(speed);
  }

  public void stopIntake() {
    var speed = 0;
    System.out.println("Intake speed:" + speed);
    m_armMotor.set(speed);
  }

}
