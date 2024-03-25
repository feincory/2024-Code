// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
//import static frc.robot.subsystems.CANLauncher.hasnote;
import static frc.robot.Constants.climberConstants.*;

//import javax.swing.text.Position;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
// //import com.ctre.phoenix6.configs.CANcoderConfiguration;
// //import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.OperatorConstants;
// import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.arm.*;
//import frc.robot.RobotContainer;

public class climber extends SubsystemBase {
   //Creates a new rotation.
   CANSparkMax m_climbmotorlead;
   CANSparkMax m_climbmotorfollower;
   Integer m_climbstep;
   Servo m_designFlaw;
   DigitalInput m_climbsensor;
   RelativeEncoder m_climbencoder;
    static boolean climberreleased;
  SparkPIDController m_climberPID;
   double kP,kI,kD,kIz,kFF, kMaxOutput, kMinOutput;
   double position;
  boolean autolineup;
  boolean climberHomed;
  boolean climberState;
  boolean climbSafe;
 public static boolean trap;
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
 //servo
 m_designFlaw = new Servo(0);
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
 climberState = false;
 climbSafe = false;
 trap = false;
 createDashboards();

 //climbtimer.start();
}

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Climber encoder", m_climbencoder.getPosition());
    SmartDashboard.putNumber("climber step",m_climbstep);
    SmartDashboard.putBoolean("climb sensor", !m_climbsensor.get());
    SmartDashboard.putBoolean("Climb Safe",climbSafe);
  SmartDashboard.putBoolean("climberreleased",climberreleased);
  SmartDashboard.putBoolean("climberHomed",climberHomed);

         if(m_climbencoder.getPosition()> 70) {
        trap = true;}
              else{
        trap = false;
      }


    // This method will be called once per scheduler run
    }

    public enum States {STOWED,RETRACT, DEPLOYED, BOTTOM}
    public States desiredState = States.STOWED;
    public Command climbstatemach() {
      return runEnd(() -> {
        
      switch(desiredState) {
        case STOWED:
          if(!m_climbsensor.get()== true && climberHomed == false && climbSafe == true) {
             m_climbencoder.setPosition(0);
            climberHomed = true;
            desiredState = States.RETRACT;
            m_climbstep = 1;
            
          }
          break;
        
          case RETRACT:
        if(climberHomed == true && climberreleased == false && climbSafe == true) {
             m_climberPID.setReference(25, CANSparkMax.ControlType.kPosition);
             climberreleased = true;
             desiredState = States.DEPLOYED;
             m_climbstep = 2;
          }
           break;
        
        
        case DEPLOYED:
        if(m_climbencoder.getPosition()>22 && climberreleased == true && climbSafe == true){
        m_climberPID.setReference(-105.5, CANSparkMax.ControlType.kPosition);
        m_climbstep = 3;
        m_designFlaw.set(kpostDeploy);
        desiredState = States.BOTTOM;
        }else{
           m_climberPID.setReference(22, CANSparkMax.ControlType.kPosition);
        }
        break;

        case BOTTOM:
        
        m_climberPID.setReference(-106.5, CANSparkMax.ControlType.kPosition);


        m_climbstep = 4;
        climbstatemach().isFinished();
        break;
      
      
      
      }}, () -> {m_climbmotorlead.set(-.01);});

      
    }
    // servo reset
    
     public void servoPreDeploy(){
      m_designFlaw.setAngle(kpreDeploy);
     }
      public void servoPostDeploy(){
      m_designFlaw.setAngle(kpostDeploy);
     }



    public void climberencoderreset(){
      if(!m_climbsensor.get()== true && climberHomed == false && climbSafe == true){
        m_climbencoder.setPosition(0);
        climberHomed = true;

      } 
      else{}
      }


    public void climbReleaseCommand() {
      if(climberHomed == true && climberreleased == false && climbSafe == true){
      m_climberPID.setReference(20, CANSparkMax.ControlType.kPosition);
    }

      if(m_climbencoder.getPosition()>16 && climberreleased == false && climbSafe == true){
      climberreleased = true;
      }

      if(climberHomed == true && climberreleased == true && climbSafe == true){
      m_climberPID.setReference(-106, CANSparkMax.ControlType.kPosition);
      m_designFlaw.set(kpostDeploy);

      }  
      else{}
      }
    

    
    public void climbwinchbottom() {
      if(climberHomed == true && !m_climbsensor.get()==false && climbSafe == true ){
      m_climberPID.setReference(72, CANSparkMax.ControlType.kPosition); //was 120 changed to accomidate rope streched
    
      }
      else{
       m_climbmotorlead.set(0);
      }


      }      
      
    public void climberReleaseable(){
       climbSafe = true;
    }
    public void climberNotReleaseable(){
      climbSafe = false;
    }

    // new code
    public void climbRetract() {
  
    if(climberHomed == true && climberreleased == false && climbSafe == true){
      m_climberPID.setReference(20, CANSparkMax.ControlType.kPosition);
    }


        if(m_climbencoder.getPosition()<16 && m_climbencoder.getPosition()>28) {
        climberreleased = true;
        }
      
      }
        
    
    public Command climbRelease() {
    return run(()->{
        if(climberHomed == true && climberreleased == true && climbSafe == true){
          m_climberPID.setReference(-106.5, CANSparkMax.ControlType.kPosition);
          m_designFlaw.set(kpostDeploy);
        }
       
       
        if(m_climbencoder.getPosition()>-104 && m_climbencoder.getPosition()<-180) {
          climbRelease().isFinished();}
        });
      }
    
          
          
    public Command climb() {
    return run(()->{
        if(climberHomed == true && !m_climbsensor.get()==false && climbSafe == true ){
      m_climberPID.setReference(72, CANSparkMax.ControlType.kPosition); //was 120 changed to accomidate rope streched
      }
      if(m_climbencoder.getPosition()>70 && m_climbencoder.getPosition()<121) {
          climb().isFinished();}});}
  
      
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
 SmartDashboard.putBoolean("Climb Safe",climbSafe);
 SmartDashboard.putBoolean("climberreleased",climberreleased);
 SmartDashboard.putBoolean("climberHomed",climberHomed);
}

public boolean climberreleased() {
    throw new UnsupportedOperationException("Unimplemented method 'climberreleased'");
}
}