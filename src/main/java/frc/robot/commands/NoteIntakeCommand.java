package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class NoteIntakeCommand extends Command {
  /** Creates a new NoteIntakeCommand. */
  
 IntakeSubsystem intakeSubsystem;
  private final double intakeSpeed = 0.8;
  BooleanSupplier leftBumper;
  BooleanSupplier rightBumper;

  public NoteIntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier leftBumper, BooleanSupplier rightBumper) {
    this.intakeSubsystem = intakeSubsystem;
    this.leftBumper = leftBumper;
    this.rightBumper = rightBumper;
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
    if(leftBumper.getAsBoolean()){
      intakeSubsystem.runIntake(intakeSpeed);
    }else if(rightBumper.getAsBoolean()){
      intakeSubsystem.runIntake(-intakeSpeed);
    }else{
      intakeSubsystem.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}