// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import static frc.robot.Constants.climberConstants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class climber extends SubsystemBase {
   //Creates a new rotation.
   CANSparkMax m_climbmotorlead;
   CANSparkMax m_climbmotorfollower;
   Integer m_climbstep;
   DigitalInput m_climbsensor;
   RelativeEncoder m_climbencoder;
   boolean climberreleased;
  SparkPIDController m_climberPID;
   double kP,kI,kD,kIz,kFF, kMaxOutput, kMinOutput;
   double position;
  boolean autolineup;
  boolean climberHomed;
  public Object cancel;  // CANcoder armCANenCaNcoder;
  public climber() {


     // defines new mototrs
  m_climbmotorfollower = new CANSparkMax(kclimberfollower, MotorType.kBrushless);
  m_climbmotorlead = new CANSparkMax(kclimberlead, MotorType.kBrushless);
  //binds motor to encoder
  m_climbencoder = m_climbmotorlead.getEncoder();
  m_climberPID= m_climbmotorlead.getPIDController();
  m_climberPID.setFeedbackDevice(m_climbencoder);
  // resets encoders
 m_climbmotorlead.restoreFactoryDefaults();
 m_climbmotorfollower.restoreFactoryDefaults();
  // set curant limet
  m_climbmotorlead.setSmartCurrentLimit(karmcurrentLimit);

  m_climbmotorlead.setOpenLoopRampRate(.4);
  m_climbmotorlead.setIdleMode(IdleMode.kBrake);

 m_climbmotorfollower.follow(m_climbmotorlead, true);
 
 m_climbstep = 0;
 m_climbsensor = new DigitalInput(0);
 
 // pid sets
 m_climberPID.setP(kPclimber);
 m_climberPID.setI(kIclimber);
 m_climberPID.setD(kDclimber);
 m_climberPID.setIZone(kIzclimber);
 m_climberPID.setFF(kFFclimer);
 m_climberPID.setOutputRange(kMinOutputclimer, kMaxOutputclimer);
 //
 //sets ranges
 position = 0;
 //m_climberPID.setReference(position, CANSparkMax.ControlType.kPosition);
 autolineup =false;
 climberHomed = false;
 climberreleased = false;
 
 createDashboards();

 //climbtimer.start();
}

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Climber encoder", m_climbencoder.getPosition());
    SmartDashboard.putNumber("climber step",m_climbstep);
    SmartDashboard.putBoolean("climb sensor", !m_climbsensor.get());

    // This method will be called once per scheduler run
    }

    // public enum States {DEFAULT, UP, DOWN}
    // private States desiredState = States.DEFAULT;
    // public Command defaultCommand() {
    //   return run(() -> {
    //     m_climbmotorlead.set(0);

    //   switch(desiredState) {
    //     case DEFAULT:
    //       if(m_climbsensor.get()) {
    //         m_climber.
    //       }
    //       break;
    //     case UP:
        
    //     break;
    //   }

    //   });
    // }
    public void climberencoderreset(){
      if(!m_climbsensor.get()== true && climberHomed == false){
        m_climbencoder.setPosition(0);
        climberHomed = true;

      } 
      else{}
      }


    public void climbReleaseCommand() {
      if(climberHomed == true && climberreleased == false){
      m_climberPID.setReference(25, CANSparkMax.ControlType.kPosition);
  
      }

      if(m_climbencoder.getPosition()>24 && climberreleased == false){
      climberreleased = true;
      }

      if(climberHomed == true && climberreleased == true){
      m_climberPID.setReference(-165, CANSparkMax.ControlType.kPosition);
      }  
      else{}
      }

    
    public void climbwinchbottom() {
      if(climberHomed == true && !m_climbsensor.get()==false){
      m_climberPID.setReference(113, CANSparkMax.ControlType.kPosition);
      }
      else{
       m_climbmotorlead.set(0);
      }
      }      
      

    
      //return runOnce(() -> m_climberPID.setReference(-10, CANSparkMax.ControlType.kPosition));}
        
    
// public Command autoclmbCommand() {
//       return runEnd(() -> {
//         if(m_climbencoder.getPosition() <= 10 /*&& m_climbsensor.get() == false*/) {
//           autolineup = true;
//         }
//         else  {}
//       }, () -> { m_climbmotorlead.set(0);});
//     }
// public Command climbrun() {
//       return runEnd(() -> {
//         if(autolineup == true /*&& m_climbsensor.get() == false*/  ) {
//           m_climberPID.setReference(10, CANSparkMax.ControlType.kPosition);

          
//         }
//         else  {}
//       }, () -> { m_climbmotorlead.set(0);});
//     }
// public Command climbNotSafe(){
//   return run(() ->{
//     if(m_climbsensor.get() == false){
//       m_climberPID.setOutputRange(kMinOutputNotSafe, kMaxOutputNotSafe);
//       climbReleaseCommand();
//     }else{m_climbmotorlead.set(0);}});
// }

// public void climbrelease(){
//   //reset encoder to start release
//   if (m_climbstep == 0 && m_climbsensor.get() != true){
//     m_climbencoder.setPosition(0);
//     m_climbstep = 1;
//   }

//   //unwinde motor for set distance
//   if (m_climbstep == 1){
//     m_climbmotorlead.set(-.2);
//     //new InstantCommand(arm.m_arm::armpositionTrapClimb);
// }
//   if(m_climbstep == 1 && m_climbencoder.getPosition() < -75){
//       m_climbstep = 2;
//     }

//     if (m_climbstep == 2){
//     m_climbmotorlead.set(0);
//     climberreleased = true;



//   }
// }

// public void  climbauto(){
//   //Climb up to trap
//   if (m_climbstep == 2 ){
//     m_climbmotorlead.set(.2);
//         if(m_climbencoder.getPosition() > 150){
//       m_climbstep = 3;
//     }

//   }

//   //Stop at "Tilt Point" and shooter starts up
//   if (m_climbstep == 3){
//     //pause for
//     m_climbmotorlead.set(0);
//     //write command to spit game piece out
//     m_climbstep = 4;
//   }

//     if (m_climbstep == 4){
//     //give time for game piece to exit shooter
//     m_climbstep = 5;
//   }

//   //Reverse back down slowly
//       if (m_climbstep == 5){
//     m_climbmotorlead.set(-.2);
//         if(m_climbencoder.getPosition() <= -8){
//       m_climbstep = 7;
//     }
//   }

// //Land down nicely
//      /* if (m_climbstep == 6){
//     m_climbmotorlead.set(0);
//     m_climbstep = 7;
//   }*/

//   //Hook Reversed (hook comes off chain)
//     if (m_climbstep == 7){
//     m_climbmotorlead.set(0);
//   m_climbstep = 8;
//   }

//   }

// //leave for manual control
// public void  climbreset(){
//     m_climbstep = 0;
// }

//leave for manual control
public void  climbupmanual(){
  m_climbmotorlead.set(-.5);//was -.5
}

public void climbdownmanual(){
m_climbmotorlead.set(.5);//was .5
}

public void stop(){
  m_climbmotorlead.set(0);
}



public void createDashboards() {
 /* ShuffleboardTab climberTab = Shuffleboard.getTab("climber");
  climberTab.addNumber("climber encoder", m_climbencoder.getPosition())
  .withSize(1,1)
  .withPosition(0,0);*/
 // SmartDashboard.putNumber("Climber encoder", m_climbencoder.getPosition());
 // SmartDashboard.putNumber("climber step",m_climbstep);
 // SmartDashboard.putBoolean("climb sensor", m_climbsensor.get());
}

public boolean climberreleased() {
  // TODO Auto-generated method stub
  throw new UnsupportedOperationException("Unimplemented method 'climberreleased'");
}
}