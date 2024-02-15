package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType; 

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
 
  private CANSparkMax m_armMotor;
  private CANSparkMax m_noteLock;
  //com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX m_armMotor = new WPI_TalonSRX(RobotMap.m_armMotorPort); change motor contgroller to SparkMax for neo 1.1 motor
  

  public IntakeSubsystem() {

  m_armMotor = new CANSparkMax (RobotMap.m_armMotorPort,MotorType.kBrushless);
    m_armMotor.setInverted(false);
  //m_noteLock = new CANSparkMax (RobotMap.m_noteLockPort,MotorType.kBrushless);
  // Create a new CANSparkBase object with device ID 4
CANSparkBase spark = new CANSparkBase (RobotMap.m_armMotorPort, MotorType.kBrushless);

// Set the idle mode to brake
spark.setIdleMode(CANSparkBase.IdleMode.kBrake);

// Set the output to 0.5 (50% duty cycle)
spark.set(0.5);

// Wait for 2 seconds
Timer.delay(2);

// Set the output to 0 (stop the motor)
spark.set(0);

// The motor will brake and resist any external motion
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    System.out.println("Intake speed:" + speed);
    double speedModifier = .5;
    m_armMotor.set(speed * speedModifier);
  }

  public void stopIntake() {
    var speed = 0;
    System.out.println("Intake speed:" + speed);
    m_armMotor.set(speed);
  }

}
