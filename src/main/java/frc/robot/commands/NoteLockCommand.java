package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteLockSubsystem;

public class NoteLockCommand extends Command {
  /** Creates a new NoteLockCommand. */
  
 NoteLockSubsystem NoteLockSubsystem;
  private final double lockSpeed = 1.0;
  BooleanSupplier xButton;
  BooleanSupplier yButton;

  public NoteLockCommand(NoteLockSubsystem noteLockSubsystem, BooleanSupplier xButton, BooleanSupplier yButton) {
    this.NoteLockSubsystem = noteLockSubsystem;
    this.xButton = xButton;
    this.yButton = yButton;
    addRequirements(noteLockSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run the Intake motors when one of the bumpers are pushed. If neither bumper is pressed then stop the motors.
    if(xButton.getAsBoolean()){
      NoteLockSubsystem.incrementPosition();
    }else if(yButton.getAsBoolean()){
      NoteLockSubsystem.decrementPosition();
    }
    // else{
    //   NoteLockSubsystem.stopLift();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NoteLockSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}