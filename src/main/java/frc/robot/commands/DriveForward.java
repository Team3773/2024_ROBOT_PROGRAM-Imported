// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends Command {
  /** Creates a new DriveForward. */

  DriveTrain driveTrain;
  double distance;
  public DriveForward(DriveTrain driveTrainSubsystem, double distanceMeters) {
    this.driveTrain = driveTrainSubsystem;
    this.distance = driveTrain.getRobotPose().getX() + distanceMeters;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(.3, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(driveTrain.getRobotPose().getX() >= this.distance){
      return true;
    }else{      
    return false;
    }
  }
}
