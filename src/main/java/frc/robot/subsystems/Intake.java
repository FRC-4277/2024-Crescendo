// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final TalonSRX intakeFront = new TalonSRX(INTAKE_FRONT);
    private final TalonSRX intakeBack = new TalonSRX(INTAKE_BACK);
  /** Creates a new Intake. */
  public Intake() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public void in() {
    move(-1);
  }
  public void out(){
    move(1);
  }
  public void move(int direction){
    intakeFront.set(TalonSRXControlMode.PercentOutput, direction * -INTAKE_SPEED);
    intakeBack.set(TalonSRXControlMode.PercentOutput, direction * INTAKE_SPEED);
  }
  public void toggle(){
   
  }
  public void stop(){ 
    intakeFront.set(TalonSRXControlMode.PercentOutput, 0);
    intakeBack.set(TalonSRXControlMode.PercentOutput, 0);
  };
}
