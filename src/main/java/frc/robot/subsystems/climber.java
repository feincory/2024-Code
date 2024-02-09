// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.climberConstants.*;
//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
   //Creates a new rotation. 
   CANSparkMax m_climbmotorlead;
   CANSparkMax m_climbmotorfollower;
  // CANcoder armCANenCaNcoder;
  public climber() {
     // defines new mototrs
  m_climbmotorfollower = new CANSparkMax(kclimberfollower, MotorType.kBrushless);
  m_climbmotorlead = new CANSparkMax(kclimberlead, MotorType.kBrushless);
  // resets encoders
 m_climbmotorlead.restoreFactoryDefaults();
 m_climbmotorfollower.restoreFactoryDefaults();
  // set curant limet
  m_climbmotorlead.setSmartCurrentLimit(karmcurrentLimit);

  m_climbmotorlead.setOpenLoopRampRate(.4);
  m_climbmotorlead.setIdleMode(IdleMode.kBrake);
  
 m_climbmotorfollower.follow(m_climbmotorlead, true);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //var armposition = rotateCANencoder.getAbsolutePosition();
  }

public void setarm(double speed){
  m_climbmotorlead.set(speed*4);
}

public void  climbup(){
  m_climbmotorlead.set(-.4);
}

public void climbdown(){
  
m_climbmotorlead.set(.4);
}

public void stop(){
  m_climbmotorlead.set(0);
}
}
