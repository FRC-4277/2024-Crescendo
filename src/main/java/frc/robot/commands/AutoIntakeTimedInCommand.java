package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntakeTimedInCommand extends Command {
  private Intake intake;
  private final Timer timer = new Timer();
  private final double runTime;
  /** Creates a new IntakeIn. */
  public AutoIntakeTimedInCommand(Intake intake, double runTime) {
    this.intake = intake;
    this.runTime = runTime;
    addRequirements(intake);
  }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.in();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= runTime;
  }
}