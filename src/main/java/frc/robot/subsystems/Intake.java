// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final TalonFX intakeFront = TalonFX(INTAKE_FRONT);
    private final TalonFX intakeBack = TalonFX(INTAKE_BACK);
  /** Creates a new Intake. */
  public Intake() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public void in() {
    move(1);
  }
  public void out(){
    move(-1);
  }
  public void move(int direction){
    intakeFront.set(1.0 * -direction);
    intakeBack.set(1.0 * direction);
  }
  public void toggle(){
   
  }
  public void stop(){ 
   intakeFront.set(0);
   intakeBack.set(0);
  };
}
