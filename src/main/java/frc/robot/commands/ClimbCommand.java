package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    /*Creates a new Climb Command */
   ClimbSubsystem ClimbSubsystem;
  private final double climbSpeed = 0.45;
  BooleanSupplier leftTriggerAxis;
  BooleanSupplier rightTriggerAxis;

  public ClimbCommand(ClimbSubsystem climbSubsystem, BooleanSupplier leftTriggerAxis, BooleanSupplier rightTriggerAxis) {
    this.ClimbSubsystem = climbSubsystem;
    this.leftTriggerAxis = leftTriggerAxis;
    this.rightTriggerAxis = rightTriggerAxis;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run the Intake motors when one of the bumpers are pushed. If neither bumper is pressed then stop the motors.
    if(leftTriggerAxis.getAsBoolean()){
      ClimbSubsystem.runLift(climbSpeed);
    }else if(rightTriggerAxis.getAsBoolean()){
      ClimbSubsystem.runLift(-climbSpeed);
    }else{
      ClimbSubsystem.stopLift();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
