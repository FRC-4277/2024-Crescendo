// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRedAmpScore extends SequentialCommandGroup {
  /** Creates a new AutonomousRedAmpScore. */
  public AutonomousRedAmpScore(DriveTrain driveTrain, Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutonomousDriveDistance(driveTrain, 0, 0.2, 2),
        new AutonomousTurn(driveTrain, 0.1, 90),
        new AutonomousDriveDistance(driveTrain, 0, 0.2, 1),
        new AutoShootOutCommandGroup(shooter, intake, 2),
        new AutonomousDriveForwardTimed(driveTrain, 3, 0.1, -0.2, 0));
  }
}
