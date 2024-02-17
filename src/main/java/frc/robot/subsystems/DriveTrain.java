// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import com.ctre.phoenix6.signals.;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.DriveTrainConstants.*;

public class DriveTrain extends SubsystemBase {
  // Talons
  private final TalonFX leftFront = new TalonFX(LEFT_FRONT);
  private final TalonFX leftBack = new TalonFX(LEFT_BACK);
  private final TalonFX rightFront = new TalonFX(RIGHT_FRONT);
  private final TalonFX rightBack = new TalonFX(RIGHT_BACK);

  // Mecanum Drive
  final MecanumDrive driveTrain = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  // NavX
  private final AHRS navx = new AHRS(SerialPort.Port.kMXP);

  //private ShuffleboardTab autoTab = Shuffleboard.getTab(AUTO_TAB);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    navx.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // Motor Configuration(s)

    rightBack.setInverted(true);
    rightFront.setInverted(true);
    leftBack.setInverted(false);
    leftFront.setInverted(false);

    this.resetEncoders();

    driveTrain.setDeadband(0.1);

  }

  
  public void driveJoystick(double x, double y, double z) {
    // drive.arcadeDrive(twistLimiter.calculate(joystick.getZ()*0.8),
    // speedLimiter.calculate(joystick.getY()));
    // driveTrain.driveCartesian( signedPow (joystick.getX(), 2) ,
    // signedPow(joystick.getY(), 2), signedPow(joystick.getZ(), 2)*.5);
    //driveTrain.driveCartesian(x, y, z);

    // option 2:
    // driveTrain.driveCartesian(x * 0.5, y * 0.5, z * 0.5);

    // option 3:
     //driveTrain.driveCartesian(getSignedPow(x, 3), getSignedPow(y, 3), getSignedPow(z, 3));

    // drive.arcadeDrive((Math.pow(joystick.getZ(),3)),
    // Math.pow(joystick.getY(),3));
    // reportToShuffleboard(joystick);

    // double adjustedZ = getAdjustedZ(joystick);
    // drive.arcadeDrive(adjustedZ, joystick.getY(), true);
  }

  private double getSignedPow(double x, double y) {
    return Math.pow(x, y);
  }

  public void driveJoystick(Joystick joystick) {
    // drive.arcadeDrive(twistLimiter.calculate(joystick.getZ()*0.8),
    // speedLimiter.calculate(joystick.getY()));
    // driveTrain.driveCartesian( signedPow (joystick.getX(), 2) ,
    // signedPow(joystick.getY(), 2), signedPow(joystick.getZ(), 2)*.5);
    //driveTrain.driveCartesian(Joystick joystick);

    // option 2:
    // driveTrain.driveCartesian(joystick.getX() * 0.5, joystick.getY() * 0.5, joystick.getZ() * 0.5);

    // option 3:
    driveTrain.driveCartesian(getSignedPow(-joystick.getY(), 3), getSignedPow(joystick.getX(), 3), getSignedPow(joystick.getZ(), 3));

    //option 1:
    // driveTrain.driveCartesian(joystick.getX(), joystick.getY(), joystick.getZ());

  }

  public void driveTimed(int direction, double speed) {
    double howToDrive = direction * speed;
    driveTrain.driveCartesian(0, howToDrive, 0);
    //driveTrain.dr
  }

  public void stop() {
    //driveTrain.driveCartesian(0,0,0);
    driveTrain.stopMotor();
  }
//Encoders
  public double getFrontLeftEncoder() {
    return leftFront.getPosition().getValueAsDouble();
  }
  public double getFrontRightEncoder() {
    return rightFront.getPosition().getValueAsDouble();
  }
  public double getBackLeftEncoder() {
    return leftBack.getPosition().getValueAsDouble();
  }
  public double getBackRightEncoder(){
    return rightBack.getPosition().getValueAsDouble();
  }
  public void resetEncoders(){
    leftFront.setPosition(0);
    rightFront.setPosition(0);
    leftBack.setPosition(0);
    rightBack.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
