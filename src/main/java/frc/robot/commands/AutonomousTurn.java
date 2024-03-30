// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutonomousTurn extends Command {
  public final DriveTrain driveTrain;
  public final double speedZ;
  public final double angle;

  /** Creates a new AutonomousDriveForward. */
  public AutonomousTurn(DriveTrain driveTrain, double speedZ, double angle) {
    this.driveTrain = driveTrain;
    this.speedZ = speedZ;
    this.angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    // this.distance = (double) driveTrain.countsGivenInches(distance);
    // this.percent = percent;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Navx reset:" );
    driveTrain.resetNavx();
    driveTrain.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveAuto(0, 0, speedZ);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Math.abs(driveTrain.getNavxAngle()) >= Math.abs(angle);
  }
}
