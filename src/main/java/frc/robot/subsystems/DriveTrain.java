// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.DriveTrainConstants.*;

public class DriveTrain extends SubsystemBase {
  // Slew Rate Limiter
  private final SlewRateLimiter filter = new SlewRateLimiter(1.2);
  private final SlewRateLimiter autoFilter = new SlewRateLimiter(1.0);
  // public static final String kCANBus = "canivore";

  // Talons
  private final TalonFX leftFront = new TalonFX(LEFT_FRONT);
  private final TalonFX leftBack = new TalonFX(LEFT_BACK);
  private final TalonFX rightFront = new TalonFX(RIGHT_FRONT);
  private final TalonFX rightBack = new TalonFX(RIGHT_BACK);

  private double processedXInput;
  private double processedYInput;
  private double processedZInput;
  private double motorRotations;
  private double numberOfWheelRotations;
  private double distanceTraveled;
  private final AHRS navx = new AHRS(SPI.Port.kMXP);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  private final CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
  private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

  // Mecanum Drive
  final MecanumDrive driveTrain = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  // NavX
  // private final Rotation2d rotation2d = new Rotation2d();

  // private ShuffleboardTab autoTab = Shuffleboard.getTab(AUTO_TAB);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // navx.reset();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // Motor Configuration(s)
    TalonFXConfiguration talonConfigurator = new TalonFXConfiguration();
    currentConfig.SupplyCurrentLimit = 35;
    currentConfig.SupplyCurrentThreshold = 40;
    currentConfig.SupplyTimeThreshold = 5;
    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimit = 35;
    currentConfig.StatorCurrentLimitEnable = true;
    feedbackConfigs.RotorToSensorRatio = 1;
    feedbackConfigs.SensorToMechanismRatio = 1;
    talonConfigurator.CurrentLimits = currentConfig;
    talonConfigurator.Feedback = feedbackConfigs;
    leftBack.getConfigurator().apply(talonConfigurator);
    leftFront.getConfigurator().apply(talonConfigurator);
    rightBack.getConfigurator().apply(talonConfigurator);
    rightFront.getConfigurator().apply(talonConfigurator);

    rightBack.setInverted(true);
    rightFront.setInverted(true);
    leftBack.setInverted(false);
    leftFront.setInverted(false);

    this.resetEncoders();
    this.resetNavx();
    this.resetGyro();
    this.setCoastMode();

    driveTrain.setDeadband(0.1);

  }

  private double getSignedPow(double x, double y) {
    return Math.pow(x, y);
  }

  public void driveAuto(double x, double y, double z) {

    // option 3:
    System.out.println("drive auto : ");

    driveTrain.driveCartesian(y, x, z < 0.1 ? z : autoFilter.calculate(z));

  
  }

  public void driveJoystick(Joystick joystick) {
   
    driveTrain.driveCartesian(getSignedPow(-joystick.getY(), 3), getSignedPow(joystick.getX(), 3),
        (Math.abs(joystick.getZ()) < 0.1) ? getSignedPow(joystick.getZ(), 3)
            : filter.calculate(getSignedPow(joystick.getZ(), 3)));

  }

  public void fieldOrientedDrive(Joystick joystick, AHRS gyro) {
    driveTrain.driveCartesian(getSignedPow(-joystick.getY(), 3), getSignedPow(joystick.getX(), 3),
        filter.calculate(getSignedPow(joystick.getZ(), 3)), gyro.getRotation2d().unaryMinus());

  }

  public void stop() {
    // driveTrain.driveCartesian(0,0,0);
    driveTrain.stopMotor();
  }

  // Encoders
  public double getFrontLeftEncoder() {
    return leftFront.getPosition().getValueAsDouble();
  }

  public double getFrontRightEncoder() {
    return rightFront.getPosition().getValueAsDouble();
  }

  public double getBackLeftEncoder() {
    return leftBack.getPosition().getValueAsDouble();
  }

  public double getBackRightEncoder() {
    return rightBack.getPosition().getValueAsDouble();
  }

  public void resetEncoders() {
    leftFront.setPosition(0);
    rightFront.setPosition(0);
    leftBack.setPosition(0);
    rightBack.setPosition(0);
  }

  public void setBrakeMode() {
    leftBack.setNeutralMode(NeutralModeValue.Brake);
    leftFront.setNeutralMode(NeutralModeValue.Brake);
    rightBack.setNeutralMode(NeutralModeValue.Brake);
    rightFront.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCoastMode() {
    leftBack.setNeutralMode(NeutralModeValue.Coast);
    leftFront.setNeutralMode(NeutralModeValue.Coast);
    rightBack.setNeutralMode(NeutralModeValue.Coast);
    rightFront.setNeutralMode(NeutralModeValue.Coast);
  }

  public void resetNavx() {
    navx.reset();
  }

  public void resetGyro() {
    gyro.calibrate();
  }

  public double getRobotDistanceTraveledFeet(double rotations) {
    //motorRotaions = Math.abs(ticks) / 2048;
    motorRotations = Math.abs(rotations);
    numberOfWheelRotations = motorRotations / 8.067;
    distanceTraveled = numberOfWheelRotations * Math.PI * 0.5;
    System.out.println("Distance traveled feet:" + distanceTraveled);
    return distanceTraveled;
  }

  public double getNavxAngle() {
    double angle = navx.getRotation2d().getDegrees();
    // double angle = gyro.getAngle();
    System.out.println("Navx angle:" + angle);
    return angle;
  }
}
