// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

import java.util.List;

public class Climber extends SubsystemBase {
  private final TalonSRX leftClimber = new TalonSRX(LEFT_CLIMBER);
  private final TalonSRX rightClimber = new TalonSRX(RIGHT_CLIMBER);
  /** Creates a new Climber. */
  public Climber() {

  //
  List<TalonSRX> motors = List.of(leftClimber, rightClimber);
  }

  public void climb(int direction){
    leftClimber.set(TalonSRXControlMode.PercentOutput, direction * 0.85);
    rightClimber.set(TalonSRXControlMode.PercentOutput, -direction * 0);
    //System.out.println("Shooter:");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
