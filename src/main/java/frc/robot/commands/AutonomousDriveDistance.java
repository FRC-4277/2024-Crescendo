// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutonomousDriveDistance extends Command {
  public final DriveTrain driveTrain;
  public final double speedX;
  public final double speedY;
  public final double distance;
  /** Creates a new AutonomousDriveForward. */
  public AutonomousDriveDistance(DriveTrain driveTrain,double speedX, double speedY, double distance) {
    this.driveTrain = driveTrain;
    this.speedX = speedX;
    this.speedY = speedY;
    this.distance = distance;

    // Use addRequirements() here to declare subsystem dependencies.
    //this.distance = (double) driveTrain.countsGivenInches(distance);
    //this.percent = percent;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveAuto(speedX, speedY, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("AutoDriveForward back left encoder: " + driveTrain.getBackLeftEncoder());
    System.out.println("AutoDriveForward distance: " + distance);
   
    return driveTrain.getRobotDistanceTraveledFeet(driveTrain.getBackLeftEncoder())>=distance;
  }
}
