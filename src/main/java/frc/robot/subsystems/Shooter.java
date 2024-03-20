// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final TalonSRX shooterTop = new TalonSRX(SHOOTER_TOP);
    private final TalonSRX shooterBottom = new TalonSRX(SHOOTER_BOTTOM);
    //private final CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
  /** Creates a new Shooter. */
  public Shooter() {
    //TalonFXConfiguration talonConfigurator = new TalonFXConfiguration();
/*currentConfig.SupplyCurrentLimit = 35;
currentConfig.SupplyCurrentThreshold = 40;
currentConfig.SupplyTimeThreshold = 1;
currentConfig.SupplyCurrentLimitEnable = true;
currentConfig.StatorCurrentLimit = 35;
currentConfig.StatorCurrentLimitEnable = true;
shooterTop.configContinuousCurrentLimit(20, 1);
talonConfigurator.CurrentLimits = currentConfig;
shooterBottom.getConfigurator().apply(talonConfigurator);
shooterTop.getConfigurator().apply(talonConfigurator);
  */ }

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
    shooterTop.set(TalonSRXControlMode.PercentOutput, direction * 0.85);
    shooterBottom.set(TalonSRXControlMode.PercentOutput, -direction * 0);
    //System.out.println("Shooter:");
  }
  public void toggle(){
   
  }
  public void stop(){ 
    shooterTop.set(TalonSRXControlMode.PercentOutput, 0);
    shooterBottom.set(TalonSRXControlMode.PercentOutput, 0);
  };
}
