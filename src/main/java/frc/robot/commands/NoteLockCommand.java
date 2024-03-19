package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteLockSubsystem;

public class NoteLockCommand extends Command {
  /** Creates a new NoteLockCommand. */
  
 NoteLockSubsystem NoteLockSubsystem;
  BooleanSupplier xButton;
  BooleanSupplier yButton;
  private final double lockedPosition = 0;
  private final double unlockedPosition = 92.05;

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
    //Run the Lock motor when X/Y are pushed. If neither button is pressed then stop the motors.
    if(xButton.getAsBoolean()){
      NoteLockSubsystem.goToPosition(lockedPosition);
    }else if(yButton.getAsBoolean()){
      NoteLockSubsystem.goToPosition(unlockedPosition);
    }
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