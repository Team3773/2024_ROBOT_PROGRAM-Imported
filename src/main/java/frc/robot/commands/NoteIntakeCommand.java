package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class NoteIntakeCommand extends Command {
  /** Creates a new NoteIntakeCommand. */
  
 IntakeSubsystem intakeSubsystem;
  private final double intakeSpeed = 0.45;
  BooleanSupplier aButton;
  BooleanSupplier bButton;

  public NoteIntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier aButton, BooleanSupplier bButton) {
    this.intakeSubsystem = intakeSubsystem;
    this.aButton = aButton;
    this.bButton = bButton;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run the Intake motors when one of the bumpers are pushed. If neither bumper is pressed then stop the motors.
    if(bButton.getAsBoolean()){
      //intakeSubsystem.(intakeSpeed);
      intakeSubsystem.incrementPosition();
    }else if(aButton.getAsBoolean()){
      intakeSubsystem.decrementPosition();
      //intakeSubsystem.runLift(-intakeSpeed);
    }else{
      //intakeSubsystem.stopLift();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}