// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.armConstants.*;

import javax.swing.text.Position;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
  //Creates a new rotation. 
   CANSparkMax m_armmotor;
   CANcoder rotateCANencoder;
   double pidout;
   double position;
   double adjustedposition;
   double kP, kI, kD, kIz, kFF ,kMaxOutput, kMinOutput;
   PIDController m_armPidController;
  
  
  // CANcoder armCANenCaNcoder;
  public arm() {
     // defines new mototrs
   m_armmotor = new CANSparkMax(karmId, MotorType.kBrushless);
  
  // resets encoders
  m_armmotor.restoreFactoryDefaults();
 // pid controller


  // set curant limet
  m_armmotor.setSmartCurrentLimit(karmcurrentLimit);
  m_armmotor.setOpenLoopRampRate(.2);
  m_armmotor.setClosedLoopRampRate(.1);
  m_armmotor.setIdleMode(IdleMode.kBrake);
  //arm CANcoder
  rotateCANencoder = new CANcoder(armCANcoderId,drivecanbusname);
  rotateCANencoder.getAbsolutePosition().setUpdateFrequency(100);
  m_armPidController = new PIDController(kP, kI, kD);


  
  
 // PID coefficients
 
 // set PID coefficients
 m_armPidController.setP(kP);
 m_armPidController.setI(kI);
 m_armPidController.setD(kD);
 
 
 






createDashboards();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   var armposition = rotateCANencoder.getAbsolutePosition();

 
  
  }

/*public void armpositionhome(){

  pidout = m_armPidController.
  m_armPidController.setSetpoint(-.5);
 m_armmotor.set(pidout);

} */
  public void armposition1(){
m_armPidController.calculate(rotateCANencoder.getAbsolutePosition().getValueAsDouble(), position);
 m_armPidController.setSetpoint(-.3);
 m_armmotor.set(pidout);

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

// creates a value for shuffle board
public double getposition(){
return rotateCANencoder.getAbsolutePosition().getValue();}
public double getcommanded(){
return m_armPidController.getSetpoint();
}
public void createDashboards() {
  ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  armTab.addNumber("Arm Position", this::getposition)
  .withSize(1,1)
  .withPosition(0,0);
  armTab.addNumber("arm commanded",this::getcommanded);
}


}
