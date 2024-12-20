package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteLockSubsystem;

public class NoteLockCommand extends Command {
  /** Creates a new NoteLockCommand. */
  
 NoteLockSubsystem NoteLockSubsystem;
  BooleanSupplier xButton;
  BooleanSupplier yButton;
  DoubleSupplier intakePosition;
  private final double lockedPosition = 0;
  private final double unlockedPosition = 92.05;
  private final double autoUnlockThreshold = 6;

  public NoteLockCommand(NoteLockSubsystem noteLockSubsystem, BooleanSupplier xButton, BooleanSupplier yButton, DoubleSupplier intakePosition) {
    this.NoteLockSubsystem = noteLockSubsystem;     
    this.xButton = xButton;
    this.yButton = yButton;
    this.intakePosition = intakePosition;
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
    
    SmartDashboard.putNumber("Current Arm Position", intakePosition.getAsDouble());
    if(intakePosition.getAsDouble() > autoUnlockThreshold){
    NoteLockSubsystem.goToPosition(unlockedPosition);
     }

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