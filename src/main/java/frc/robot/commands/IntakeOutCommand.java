package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeOutCommand extends Command {
  private Intake intake;
  /** Creates a new IntakeIn. */
  public IntakeOutCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.out();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}