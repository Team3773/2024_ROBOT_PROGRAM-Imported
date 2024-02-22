package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
  /* Creates a new Climb Command */
  ClimbSubsystem climbSubsystem;
  private final double climbSpeed = 0.8;
  BooleanSupplier bumper, trigger;
  // BooleanSupplier leftTrigger;

  public ClimbCommand(ClimbSubsystem climbSubsystem, BooleanSupplier bumper, BooleanSupplier trigger) {
    this.climbSubsystem = climbSubsystem;
    this.bumper = bumper;
    this.trigger = trigger;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run the lift motors when one of the bumpers are pushed. If neither bumper or
    // trigger is pressed then stop the motors.
    runLift(climbSubsystem, bumper.getAsBoolean(), trigger.getAsBoolean());
  }

  private void runLift(ClimbSubsystem subsystem, boolean bumper, boolean trigger) {
    if (bumper) {
      subsystem.runLift(climbSpeed);
    } else if (trigger) {
      subsystem.runLift(-climbSpeed);
    } else {
      subsystem.stopLift();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
