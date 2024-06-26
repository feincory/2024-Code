// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.CAN;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANLauncher extends SubsystemBase {
  CANSparkMax m_launchWheel;
  CANSparkMax m_feedWheel;
  CANSparkMax m_kickerWheel;
  public DigitalInput m_ringDetect;
  public boolean hasnote;
  RelativeEncoder m_intakeencode;
  

  /** Creates a new Launcher. */
  public CANLauncher() {
    m_launchWheel = new CANSparkMax(kLauncherID, MotorType.kBrushless);
    m_feedWheel = new CANSparkMax(kFeederID, MotorType.kBrushless);
    m_kickerWheel = new CANSparkMax(kKickerID, MotorType.kBrushless);
   m_ringDetect = new DigitalInput(1);
    m_launchWheel.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
    m_kickerWheel.setSmartCurrentLimit(kFeedCurrentLimit);
    m_launchWheel.restoreFactoryDefaults();
    m_feedWheel.restoreFactoryDefaults();
    m_kickerWheel.restoreFactoryDefaults();
   m_ringDetect.get();
   m_intakeencode = m_feedWheel.getEncoder();
   m_kickerWheel.setOpenLoopRampRate(.2);
   createDashboards();
  }
 
  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederSpeed);
          setLaunchWheel(kIntakeLauncherSpeed);
          setKickerWheel(kIntakeKickerSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
 
   
  
  public void noteMoveForAmp(){
    m_feedWheel.set(.5);
  }
  public void noteMoveForshot(){
    m_feedWheel.set(-.2);
    m_launchWheel.set(.4);
  }

  public Command getReverseNoteCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederReverseSpeed);
          setLaunchWheel(kLauncherReverseSpeed);
          setKickerWheel(-kIntakeKickerSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }
 public void setKickerWheel(double speed) {
    m_kickerWheel.set(speed);
  }
  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
    m_kickerWheel.set(0);
  }
  // creates a value for shuffle board
public boolean getstate(){
  return m_ringDetect.get();}


  public void createDashboards() {
  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  shooterTab.add("ring detect", getstate());
  
}
    
  }
  

