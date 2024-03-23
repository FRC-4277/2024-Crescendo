// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX intakeFront = new WPI_TalonSRX(INTAKE_FRONT);
    private final WPI_TalonSRX intakeBack = new WPI_TalonSRX(INTAKE_BACK);
    private DigitalInput intakeStopSensor = new DigitalInput(0);
  /** Creates a new Intake. */
  public Intake() {
    intakeFront.configPeakCurrentLimit(35, 1000);
    intakeBack.configPeakCurrentLimit(35,1000);
    intakeFront.configContinuousCurrentLimit(30);
    intakeBack.configContinuousCurrentLimit(30);
    intakeFront.enableCurrentLimit(true);
    intakeBack.enableCurrentLimit(true);
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
  }
  public boolean getStopSensor(){
    return intakeStopSensor.get();
  }
}
