// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix6.hardware.CANcoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
  // joysticks
  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;
    }

/*public static class shooterConstants{ 
  // motor can ids 
[]\
    public static final int kshooterId = 2;
    public static final int kintakeId = 1;
  //curent limiting
    public static final int kshootercurrentLimit = 40;
    public static final int kintakecurrentLimit = 40;
  // speeds sets
  public static final double kintakespeed = .5;
  public static final double koutakespeed = -.5;
  public static final double kshooterspeed = .5;
  // delay
  public static final double kshootdelay =1;
  }*/
  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 1;
    public static final int kLauncherID = 2;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 40;
    public static final int kFeedCurrentLimit = 40;


    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = -.8;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = 0;
    public static final double kIntakeFeederSpeed = .7;
    public static final double kLauncherReverseSpeed = 0.4;
    public static final double kIntakeFeederReverseSpeed = -.25;

    public static final double kLauncherDelay = 1;
   
  }

public static class armConstants{
  // motor can ids
  public static final int karmId = 3;
  // curant limiting
  public static final int karmcurrentLimit = 60;
    //rotate cancoder 
  public static final String drivecanbusname = "Drive CAN";
  public static final int armCANcoderId = 47;
  public static double kP = .4;
  public static double kI =.2;
  public static double kD = .1;
  public static double kIz = .15;
  
  public static double kMaxOutput=.1;
  public static double kMinOutput=-.1;
  public static double kCPR = 8192;
  public static double koffset = -.1;
}


public static class climberConstants{
// motor can ids
public static final int kclimberlead = 4;
public static final int kclimberfollower = 5;
// curant limiting
public static final int karmcurrentLimit = 60;

}


}
