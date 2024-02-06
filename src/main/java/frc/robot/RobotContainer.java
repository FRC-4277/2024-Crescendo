// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveManualCommand;
import frc.robot.commands.IntakeInCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.ShooterShootOutCommand;
import frc.robot.commands.ShooterReverseCommand;

import static frc.robot.Constants.Controllers.*;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Shuffleboard
  private final ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
  private final ShuffleboardTab controlsTab = Shuffleboard.getTab("Main");
  private SendableChooser autoChooser;

  // subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  // TODO:  Add climber

  // TODO:  Add Limelight

  // Controllers
  private final XboxController controller = new XboxController(XBOX_CONTROLLER);
  private final Joystick joystick = new Joystick(LOGITECH_JOYSTICK);

  // Commands
  private final DriveManualCommand driveManualCommand = new DriveManualCommand(driveTrain, joystick, controller);
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intake);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intake);
  private final ShooterShootOutCommand shootCommand = new ShooterShootOutCommand(shooter);
  private final ShooterReverseCommand shooterReverseCommand = new ShooterReverseCommand(shooter);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveTrain.setDefaultCommand(driveManualCommand);
    setUpSmartDashboard();
  }

  private void configureBindings() {
    // Put all the button controlles here

    // Intake
    // Intake In - X Button
    new JoystickButton(controller, Button.kX.value)
        .whileTrue(intakeInCommand);

    new JoystickButton(joystick, Button.kX.value)
        .whileTrue(intakeInCommand);

    // Intake Out - Y Button

    new JoystickButton(controller, Button.kY.value)
        .whileTrue(intakeOutCommand);

    new JoystickButton(joystick, 5)
        .whileTrue(intakeOutCommand);

    //Shooter
    // Shooter shoot - B Button
    new JoystickButton(controller, Button.kB.value)
        .whileTrue(shootCommand);

    // Shooter reverse - A Button
    new JoystickButton(controller, Button.kA.value)
        .whileTrue(shooterReverseCommand);

    // Climber
    // TODO:  Add commands
    
  }

  private void setUpSmartDashboard() {
    controlsTab.add(driveTrain);
    controlsTab.add(intake);
    controlsTab.add(shooter);

    setupDriverTab();

  }

   private void setupDriverTab() {
    // TODO:  Add stuff like game time, camera stream, etc
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Autos();
  }
}
