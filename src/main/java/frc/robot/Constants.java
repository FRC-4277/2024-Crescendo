// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Controllers {
      public static final int XBOX_CONTROLLER = 0;
      public static final int LOGITECH_JOYSTICK = 1;
    }

    public static class DriveTrainConstants {

      public static final int LEFT_FRONT = 4;
      public static final int LEFT_BACK = 3;
      public static final int RIGHT_FRONT = 2;
      public static final int RIGHT_BACK = 1;

    }
    public static class IntakeConstants {
      public static final int INTAKE_BACK = 21;
      public static final int INTAKE_FRONT = 30;
      public static final double INTAKE_SPEED = 0.95; 
      
    }
    public static class ShooterConstants {
      public static final int SHOOTER_BOTTOM = 23;
      public static final int SHOOTER_TOP = 31;
      public static final double SHOOTER_SPEED = 0.55;
        
      }

    public static class ClimberConstants {
      public static final int LEFT_CLIMBER = 61;
      public static final int RIGHT_CLIMBER = 22;
    }
    
    public static class ShuffleboardConstants {
      public static final String AUTO_TAB = "Autonomous";
    }
}
