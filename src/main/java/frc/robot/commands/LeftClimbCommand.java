package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftClimbSubsystem;

public class LeftClimbCommand extends Command {
    /*Creates a new Climb Command */
  LeftClimbSubsystem LeftClimbSubsystem;
  private final double climbSpeed = 0.45;
  BooleanSupplier leftBumper;
  DoubleSupplier leftTrigger;

  public LeftClimbCommand(LeftClimbSubsystem leftclimbSubsystem, BooleanSupplier leftBumper, DoubleSupplier leftTrigger) {
    this.LeftClimbSubsystem = leftclimbSubsystem;
    this.leftBumper = leftBumper;
    this.leftTrigger = leftTrigger;
    addRequirements(leftclimbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run the lift motors when one of the bumpers are pushed. If neither bumper or trigger is pressed then stop the motors.
    if(leftBumper.getAsBoolean()){
      LeftClimbSubsystem.runLift(climbSpeed);
    }else if(((BooleanSupplier) leftTrigger).getAsBoolean()){
      LeftClimbSubsystem.runLift(-climbSpeed);
    }else{
      LeftClimbSubsystem.stopLift();
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LeftClimbSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
