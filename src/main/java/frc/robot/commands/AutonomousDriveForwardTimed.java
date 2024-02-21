// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutonomousDriveForwardTimed extends Command {
  private final DriveTrain driveTrain;
  private final Timer timer = new Timer();
  private final double runTime; 
  /** Creates a new AutonomousDriveForwardTimed. */
  public AutonomousDriveForwardTimed(DriveTrain driveTrain, double runTime) {
    this.driveTrain = driveTrain;
    this.runTime = runTime;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Here!");
    driveTrain.driveTimed(1,0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  timer.get() >= runTime;
  }
}
