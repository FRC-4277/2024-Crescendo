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
  private final TalonFX leftClimber = new TalonFX(LEFT_CLIMBER);
  private final TalonFX rightClimber = new TalonFX(RIGHT_CLIMBER);

  /** Creates a new Climber. */
  public Climber() {
  }

  public void up() {
    move(-1, 0.3);
  }

  public void down() {
    move(1, 0.3);
  }

  public void move(double direction, double speed) {
    leftClimber.set(direction * speed);
    rightClimber.set(direction * speed);
    // System.out.println("Shooter:");
  }

  public void stop() {
    leftClimber.set(0);
    rightClimber.set(0);
    // System.out.println("Shooter:");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
