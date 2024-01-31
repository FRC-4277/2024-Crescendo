// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.DriveTrainConstants.*;


public class DriveTrain extends SubsystemBase {
      //Talons
      private final TalonFX leftFront = new TalonFX(LEFT_FRONT);
      private final TalonFX leftBack = new TalonFX(LEFT_BACK);
      private final TalonFX rightFront = new TalonFX(RIGHT_FRONT);
      private final TalonFX rightBack = new TalonFX(RIGHT_BACK);
      
      //Mecanum Drive
      final MecanumDrive driveTrain = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);


      //NavX
      private final AHRS navx = new AHRS(SerialPort.Port.kMXP);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftBack.setInverted(true);
    rightBack.setInverted(true);


  }

  public void driveJoystick(Joystick joystick) {
   //drive.arcadeDrive(twistLimiter.calculate(joystick.getZ()*0.8), speedLimiter.calculate(joystick.getY()));
    //driveTrain.driveCartesian( signedPow (joystick.getX(), 2) , signedPow(joystick.getY(), 2), signedPow(joystick.getZ(), 2)*.5);
    driveTrain.driveCartesian((joystick.getX()), (joystick.getY()), (joystick.getZ()));

    //drive.arcadeDrive((Math.pow(joystick.getZ(),3)), Math.pow(joystick.getY(),3));
    //reportToShuffleboard(joystick);

   // double adjustedZ = getAdjustedZ(joystick);
   // drive.arcadeDrive(adjustedZ, joystick.getY(), true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
