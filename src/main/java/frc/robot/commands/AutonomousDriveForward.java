// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutonomousDriveForward extends Command {
  public final DriveTrain driveTrain;
  public final int direction;
  public final double speed;
  public final double distance;
  /** Creates a new AutonomousDriveForward. */
  public AutonomousDriveForward(DriveTrain driveTrain, int direction, double speed, double distance) {
    this.driveTrain = driveTrain;
    this.direction = direction;
    this.speed = speed;
    this.distance = distance;

    // Use addRequirements() here to declare subsystem dependencies.
    //this.distance = (double) driveTrain.countsGivenInches(distance);
    //this.percent = percent;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveJoystick(speed * direction, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
