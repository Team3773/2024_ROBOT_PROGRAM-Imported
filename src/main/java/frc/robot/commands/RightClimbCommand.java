package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightClimbSubsystem;

public class RightClimbCommand extends Command {
    /*Creates a new Climb Command */
  RightClimbSubsystem RightClimbSubsystem;
  private final double climbSpeed = 0.45;
  BooleanSupplier rightBumper;
  DoubleSupplier rightTrigger;

  public RightClimbCommand(RightClimbSubsystem rightclimbSubsystem, BooleanSupplier rightBumper, DoubleSupplier rightTrigger) {
    this.RightClimbSubsystem = rightclimbSubsystem;
    this.rightBumper = rightBumper;
    this.rightTrigger = rightTrigger;
    addRequirements(rightclimbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run the lift motors when one of the bumpers are pushed. If neither bumper or trigger is pressed then stop the motors.
    if(rightBumper.getAsBoolean()){
      RightClimbSubsystem.runLift(climbSpeed);
    }else if(((BooleanSupplier) rightTrigger).getAsBoolean()){
      RightClimbSubsystem.runLift(-climbSpeed);
    }else{
      RightClimbSubsystem.stopLift();
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RightClimbSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

