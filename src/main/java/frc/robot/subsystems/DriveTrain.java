// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;


/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class DriveTrain extends SubsystemBase {
  /*
   * Class member variables. These variables represent things the class needs to
   * keep track of and use between
   * different method calls.
   */

  private double kGearRatio = 8.46;;
  private double kWheelDiameter = Units.inchesToMeters(6);;
  DifferentialDrive m_drivetrain;

  WPI_TalonSRX leftFront;
  WPI_TalonSRX leftRear;
  WPI_TalonSRX rightFront;
  WPI_TalonSRX rightRear;

  // Pigeon2 pigeon = new Pigeon2(kPigeonID);
  DifferentialDriveOdometry odometry;

  /*
   * Constructor. This method is called when an instance of the class is created.
   * This should generally be used to set up
   * member variables and perform any configuration or set up necessary on
   * hardware.
   */
  public DriveTrain() {
    // pigeon.setYaw(0);
    leftFront = new WPI_TalonSRX(RobotMap.m_leftMotorFollowerPort);
    leftRear = new WPI_TalonSRX(RobotMap.m_leftMotorPort);
    rightFront = new WPI_TalonSRX(RobotMap.m_rightMotorFollowerPort);
    rightRear = new WPI_TalonSRX(RobotMap.m_rightMotorPort);

    /*
     * Sets current limits for the drivetrain motors. This helps reduce the
     * likelihood of wheel spin, reduces motor heating
     * at stall (Drivetrain pushing against something) and helps maintain battery
     * voltage under heavy demand
     */
    TalonSRXConfiguration motorConfig = generateSRXDriveMotorConfig();
    leftFront.configAllSettings(motorConfig);
    leftRear.configAllSettings(motorConfig);
    rightFront.configAllSettings(motorConfig);
    rightRear.configAllSettings(motorConfig);

    leftFront.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));
    leftRear.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));
    rightFront.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));
    rightRear.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1));

    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control
    // all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    odometry = new DifferentialDriveOdometry(new Rotation2d(), getDistanceMeters(true), getDistanceMeters(false));
  }

  private double getDistanceMeters(boolean left) {
    if (left) {
      return leftFront.getSelectedSensorPosition() / 4096.0 * kGearRatio * Math.PI * kWheelDiameter;
    } else {
      return rightFront.getSelectedSensorPosition()
          / 4096.0
          * kGearRatio
          * Math.PI
          * kWheelDiameter;
    }
  }

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          .24,
          0,
          0);


  public void resetEncoders(){
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

    public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
     // set (ControlMode mode, double demand0, DemandType demand1Type, double demand1)
      leftFront.set(
          TalonSRXControlMode.Position,
          m_feedforward.calculate(left.position));
      rightFront.set(
          TalonSRXControlMode.Position,
          m_feedforward.calculate(right.position));
  }

  /**
   * Returns the left encoder distance.
   *
   * @return the left encoder distance
   */
  public double getLeftEncoderDistance() {
    return getDistanceMeters(true);
  }

  /**
   * Returns the right encoder distance.
   *
   * @return the right encoder distance
   */
  public double getRightEncoderDistance() {
    return getDistanceMeters(false);
  }
  /*
   * Method to control the drivetrain using arcade drive. Arcade drive takes a
   * speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses
   * these to control the drivetrain motors
   */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void resetGyro() {
    //pigeon.setYaw(0);
    // TODO: Figure out how to set accum z angle
    // TODO: Reset odometry
  }

  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    /*
     * This method will be called once per scheduler run. It can be used for running
     * tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't
     * have anything to put here
     */
    odometry.update(new Rotation2d(), getDistanceMeters(true), getDistanceMeters(false));
  }

  public static TalonSRXConfiguration generateSRXDriveMotorConfig() {
    TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

    // motorConfig.Feedback.FeedbackSensorSource =
    // FeedbackSensorSourceValue.RotorSensor;
    // motorConfig.Slot0.kV = 0.1185;
    // motorConfig.Slot0.kP = 0.24;
    // motorConfig.Slot0.kI = 0.0;
    // motorConfig.Slot0.kD = 0.0;

    // motorConfig.Voltage.PeakForwardVoltage = 12;
    // motorConfig.Voltage.PeakReverseVoltage = -12;

    // motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    // motorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    // motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    // motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TO
    // // DO adjust this later
    // motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust
    // this later

    // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.slot0.kP = 0.24;
    motorConfig.slot0.kI = 0.0;
    motorConfig.slot0.kD = 0.0;

    // motorConfig.peakCurrentLimit = 35;
    return motorConfig;
  }

  // @Override
  // public void close() {
  // leftRear.close();
  // rightRear.close();
  // }
}

// public class DriveTrain extends SubsystemBase {
// PIDController leftMotorPIDController, rightMotorPIDController,
// driveGyroPIDController;
// double kP = 0.03; // Start with P = 10% of your max output, double until you
// get a quarter-decay
// // oscillation
// double kI = 0; // Start with I = P / 100
// double kD = 0; // Start with D = P * 10
// double period = 0.01;

// public WPI_TalonSRX[] driveMotors = {
// new WPI_TalonSRX(RobotMap.m_leftMotorPort),
// new WPI_TalonSRX(RobotMap.m_leftMotorFollowerPort),
// new WPI_TalonSRX(RobotMap.m_rightMotorPort),
// new WPI_TalonSRX(RobotMap.m_rightMotorFollowerPort)
// };

// DifferentialDrive robotDrive = new DifferentialDrive(driveMotors[0],
// driveMotors[2]);

// public AHRS Gyro;

// public DriveTrain() {
// super("Drive Train");

// // Set Motor Controller Control Mode
// driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
// driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());

// // Set Motor Controller Feedback Device
// driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
// 0);
// driveMotors[2].setSensorPhase(true);
// driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
// 0);

// // Set Motor Controller Peak Output Voltages & Set Motors to Coast
// for (int i = 0; i < 4; i++) {
// driveMotors[i].configPeakOutputForward(1, 0);
// driveMotors[i].configPeakOutputReverse(-1, 0);
// driveMotors[i].setInverted(true);
// driveMotors[i].enableVoltageCompensation(true);
// driveMotors[i].configVoltageCompSaturation(12, 0);
// // driveMotors[i].setSafetyEnabled(true);
// // driveMotors[i].configContinuousCurrentLimit(40, 0);
// // driveMotors[i].configPeakCurrentLimit(80, 0);
// // driveMotors[i].configPeakCurrentDuration(100, 0);
// }

// try {
// Gyro = new AHRS(SPI.Port.kMXP);

// if (Gyro == null)
// SmartDashboard.putBoolean("Gyro Detected", false);
// else
// SmartDashboard.putBoolean("Gyro Detected", true);
// } catch (Exception e) {
// DriverStation.reportError("4201 Error: Gyro not detected!", false);
// SmartDashboard.putBoolean("Gyro Detected", false);
// }
// }
// // Put methods for controlling this subsystem
// // here. Call these from Commands.

// public double getLeftEncoderValue() {
// return driveMotors[0].getSelectedSensorPosition(0);
// }

// public double getRightEncoderValue() {
// return driveMotors[2].getSelectedSensorPosition(0);
// }

// public void resetSensors() {
// driveMotors[0].setSelectedSensorPosition(0, 0, 0);
// driveMotors[2].setSelectedSensorPosition(0, 0, 0);
// try {
// Gyro.reset();
// } catch (Exception e) {

// }
// }

// public void setMotorsToBrake() {
// for (int i = 0; i < driveMotors.length; i++)
// driveMotors[i].setNeutralMode(NeutralMode.Brake);
// }

// public void setMotorsToCoast() {
// for (int i = 0; i < driveMotors.length; i++)
// driveMotors[i].setNeutralMode(NeutralMode.Coast);
// }

// public void setDriveOutput(double throttle, double angularPower) {
// double leftPWM = throttle + angularPower;
// double rightPWM = throttle - angularPower;

// if (rightPWM > 1.0) {
// leftPWM -= rightPWM - 1.0;
// rightPWM = 1.0;
// } else if (rightPWM < -1.0) {
// leftPWM -= rightPWM + 1.0;
// rightPWM = -1.0;
// } else if (leftPWM > 1.0) {
// rightPWM -= leftPWM - 1.0;
// leftPWM = 1.0;
// } else if (leftPWM < -1.0) {
// rightPWM -= leftPWM + 1.0;
// leftPWM = -1.0;
// }

// setDirectDriveOutput(leftPWM, rightPWM);
// }

// public void PIDDrive(double leftOutput, double rightOutput) {
// double leftPWM = leftOutput;
// double rightPWM = rightOutput;

// if (rightPWM > 1.0) {
// leftPWM -= (rightPWM - 1.0);
// rightPWM = 1.0;
// } else if (rightPWM < -1.0) {
// leftPWM += (-rightPWM - 1.0);
// rightPWM = -1.0;
// } else if (leftPWM > 1.0) {
// rightPWM -= (leftPWM - 1.0);
// leftPWM = 1.0;
// } else if (leftPWM < -1.0) {
// rightPWM += (-leftPWM - 1.0);
// leftPWM = -1.0;
// }

// robotDrive.tankDrive(leftPWM, rightPWM);
// }

// public void setDirectDriveOutput(double leftOutput, double rightOutput) {

// robotDrive.tankDrive(leftOutput, rightOutput);
// }

// public void setDirectDriveOutputInverted(double leftOutput, double
// rightOutput) {
// robotDrive.tankDrive(rightOutput, leftOutput);
// }

// public void setDirectDriveScaledOutput(double leftOutput, double rightOutput)
// {
// double max = Math.max(leftOutput, rightOutput);

// if (max > 1) {
// if (leftOutput == max) {
// rightOutput = rightOutput / leftOutput;
// leftOutput = 1;
// } else {
// leftOutput = leftOutput / rightOutput;
// rightOutput = 1;
// }
// }

// robotDrive.tankDrive(leftOutput, rightOutput);
// }

// public void cheesyDrive(double xSpeed, double zRotation, boolean QuickTurn) {
// robotDrive.curvatureDrive(xSpeed, zRotation, QuickTurn);
// }

// public void setTankDrive(double leftSpeed, double rightSpeed) {
// robotDrive.tankDrive(leftSpeed, rightSpeed);
// }

// public void setArcadeDrive(double speed, double turn) {
// robotDrive.arcadeDrive(speed, turn);
// }

// public void updateSmartDashboard() {
// // Use Shuffleboard to place things in their own tabs
// SmartDashboard.putNumber("Drive TrainLeft Encoder Count",
// getLeftEncoderValue());
// SmartDashboard.putNumber("Drive Train Right Encoder Count",
// getRightEncoderValue());

// SmartDashboard.putNumber("Pathfinder Left Encoder Count",
// getLeftEncoderValue());
// SmartDashboard.putNumber("Pathfinder Right Encoder Count",
// getRightEncoderValue());
// // Use SmartDashboard to put only the important stuff for drivers
// try {
// SmartDashboard.putNumber("Drive Train Gyro", Gyro.getAngle());
// SmartDashboard.putNumber("Pathfinder Gyro", Gyro.getAngle());
// // Shuffleboard.putData("Drive Train", "P Gyro", Gyro);
// // Shuffleboard.putData("Pathfinder", "DT Gyro", Gyro);
// SmartDashboard.putData(Gyro); // Check if this needs to be in this loop?
// } catch (Exception e) {

// }
// }

// public void initDefaultCommand() {
// // Set the default command for a subsystem here.
// setDefaultCommand(new SetSplitArcadeDrive());
// }
// }