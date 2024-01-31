// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;

public class DriveManualCommand extends SubsystemBase {
  /** Creates a new DriveManualCommand. */
   private final DriveTrain driveTrain;
   private final XboxController controller;
   private final Joystick joystick;

   public DriveManualCommand(DriveTrain driveTrain, Joystick joystick, XboxController controller) {
    this.driveTrain = driveTrain;
    this.joystick = joystick;
    this.controller = controller;
  }
  
  public void execute() {
      driveTrain.driveJoystick(joystick);
  }
  // Called once the command ends or is interrupted.
  
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}