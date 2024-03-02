// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;

public class TeleopDrive extends Command {
  /** Creates a new TeleopDrive. */
  DriveTrain m_DriveSubsystem;
  DoubleSupplier xAxis;
  DoubleSupplier yAxis;
  DoubleSupplier throttleSupplier;
  double throttle;

  public TeleopDrive(DriveTrain driveSubsystem, DoubleSupplier xAxis, DoubleSupplier yAxis) {
    this.m_DriveSubsystem = driveSubsystem;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.arcadeDrive(xAxis.getAsDouble(), yAxis.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
