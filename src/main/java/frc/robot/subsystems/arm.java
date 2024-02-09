// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.armConstants.*;
//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
   //Creates a new rotation. 
   CANSparkMax m_armmotor;
  // CANcoder armCANenCaNcoder;
  public arm() {
     // defines new mototrs
   m_armmotor = new CANSparkMax(karmId, MotorType.kBrushless);
  
  // resets encoders
  m_armmotor.restoreFactoryDefaults();
  // set curant limet
  m_armmotor.setSmartCurrentLimit(karmcurrentLimit);
  m_armmotor.setOpenLoopRampRate(.4);
  m_armmotor.setIdleMode(IdleMode.kBrake);
  //arm CANcoder
  //CANcoder armCANencoder = new CANcoder(armCANcoderId,drivecanbusname);
  //armCANencoder.getAbsolutePosition().setUpdateFrequency(100);


}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //var armposition = rotateCANencoder.getAbsolutePosition();
  }

public void setarm(double speed){
  m_armmotor.set(speed*4);
}

public void  armfoward(){
  m_armmotor.set(.4);
}

public void armreverese(){
  
m_armmotor.set(-.4);
}

public void stop(){
  m_armmotor.set(0);
}
}
