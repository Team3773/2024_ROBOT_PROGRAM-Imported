// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteLockSubsystem;
import frc.robot.commands.NoteLockCommand;

/** Add your docs here. */
public class RobotContainer {

        private static BooleanSupplier getBooleanSupplier(DoubleSupplier doubleSupplier) {
                return () -> doubleSupplier.getAsDouble() > 0;
        }

        // This is where robot subsystems are initialized.
        // public final DriveSubsystem m_drive = new DriveSubsystem();
        public final IntakeSubsystem m_arm = new IntakeSubsystem();
        public final ClimbSubsystem m_LeftClimbSubsystem = new ClimbSubsystem(RobotMap.m_leftLiftMotorPort, false,
                        "Left Lift ");
        public final ClimbSubsystem m_RightClimbSubsystem = new ClimbSubsystem(RobotMap.m_rightLiftMotorPort, true,
                        "Right Lift ");
        public final NoteLockSubsystem m_lock = new NoteLockSubsystem(RobotMap.m_lockMotorPort, false, "Lock Motor ");

        public final DriveTrain driveTrain = new DriveTrain();

        // public final RightClimbSubsystem m_rightclimb = new RightClimbSubsystem();

        // Define xbox controller with port mapping.
        private XboxController controller = new XboxController(0);
        private XboxController controller2 = new XboxController(1);

        // Constructor for the RobotContainer Class.
        public RobotContainer() {
                // Set Default Command for the drivetrain subsystem. This will be active during
                // teleop mode.
                System.out.println("Creating Commands");
                driveTrain.setDefaultCommand(new TeleopDrive(
                                driveTrain,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX()));

                // Set Default Command for the Note Intake Subsystem
                m_arm.setDefaultCommand(new NoteIntakeCommand(
                                m_arm,
                                () -> controller2.getAButton(),
                                () -> controller2.getBButton()));

                m_LeftClimbSubsystem.setDefaultCommand(new ClimbCommand(
                                m_LeftClimbSubsystem,
                                () -> controller2.getLeftBumper(),
                                getBooleanSupplier(() -> controller2.getLeftTriggerAxis())));

                m_RightClimbSubsystem.setDefaultCommand(new ClimbCommand(
                                m_RightClimbSubsystem,
                                () -> controller2.getRightBumper(),
                                getBooleanSupplier(() -> controller2.getRightTriggerAxis())));

                m_lock.setDefaultCommand(new NoteLockCommand(
                                m_lock,
                                () -> controller2.getXButton(),
                                () -> controller2.getYButton(),
                                () -> m_arm.getCurrentPosition()));
        }
}
