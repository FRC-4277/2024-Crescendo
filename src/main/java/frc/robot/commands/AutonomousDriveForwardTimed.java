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
  private final double x;
  private final double y;
  private final double z;
  /** Creates a new AutonomousDriveForwardTimed. */
  public AutonomousDriveForwardTimed(DriveTrain driveTrain, double runTime, double x, double y, double z) {
    this.driveTrain = driveTrain;
    this.runTime = runTime;
    this.x = x;
    this.y = y;
    this.z = z;
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
    System.out.println("AutoDriveForwardTimed!");
    driveTrain.driveAuto(x, y, z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  timer.get() >= runTime;
  }
}
