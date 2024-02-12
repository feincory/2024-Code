// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.armConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class arm extends SubsystemBase {
  //Creates a new rotation. 
  private static final String canBusName = TunerConstants.kCANbusName;
  private final TalonFX m_fx = new TalonFX(20, canBusName);
  private final CANcoder m_cc = new CANcoder(47, canBusName);

  private final StatusSignal<Boolean> f_fusedSensorOutOfSync = m_fx.getFault_FusedSensorOutOfSync();
  private final StatusSignal<Boolean> sf_fusedSensorOutOfSync = m_fx.getStickyFault_FusedSensorOutOfSync();
  private final StatusSignal<Boolean> f_remoteSensorInvalid = m_fx.getFault_RemoteSensorDataInvalid();
  private final StatusSignal<Boolean> sf_remoteSensorInvalid = m_fx.getStickyFault_RemoteSensorDataInvalid();

  private final StatusSignal<Double> fx_pos = m_fx.getPosition();
  private final StatusSignal<Double> fx_vel = m_fx.getVelocity();
  private final StatusSignal<Double> cc_pos = m_cc.getPosition();
  private final StatusSignal<Double> cc_vel = m_cc.getVelocity();
  private final StatusSignal<Double> fx_rotorPos = m_fx.getRotorPosition();
  private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);
  private int printCount = 0;
  private final StaticBrake m_brake = new StaticBrake();
  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionDutyCycle m_dutyPosition = new PositionDutyCycle(0, 0, true, 0, 0, false, false, false);

//soft limits
  //SoftwareLimitSwitchConfigs armlimit = new ForwardsSoftLimitEnable();

  
  
  // CANcoder armCANenCaNcoder;
  public arm() {
     // defines new mototrs
    /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = -0.04;
    m_cc.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 135;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .25;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold= -.164; 
    fx_cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
    fx_cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .15;
    // pid config
    fx_cfg.Slot0.GravityType =GravityTypeValue.Arm_Cosine;   
    fx_cfg.Slot0.kP = kPc;
    fx_cfg.Slot0.kD = kDc;
    fx_cfg.Slot0.kI = kIc;
    fx_cfg.Slot0.kG = kGc;

    fx_cfg.Voltage.PeakForwardVoltage =13;
    fx_cfg.Voltage.PeakReverseVoltage =-13;
    m_fx.getConfigurator().apply(fx_cfg);
    
   StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(fx_cfg);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    m_fx.setNeutralMode(NeutralModeValue.Brake);
 
  






createDashboards();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
       if (printCount++ > 10) {
      printCount = 0;
      // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
      f_fusedSensorOutOfSync.refresh();
      sf_fusedSensorOutOfSync.refresh();
      f_remoteSensorInvalid.refresh();
      sf_remoteSensorInvalid.refresh();
      boolean anyFault = sf_fusedSensorOutOfSync.getValue() || sf_remoteSensorInvalid.getValue();
      if(anyFault) {
        System.out.println("A fault has occurred:");
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync live-faulted");
        } else if (sf_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync sticky-faulted");
        }
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_remoteSensorInvalid.getValue()) {
          System.out.println("Missing remote sensor live-faulted");
        } else if (sf_remoteSensorInvalid.getValue()) {
          System.out.println("Noah is a turd");
        }
      }

      /* Print out current position and velocity */
      fx_pos.refresh(); fx_vel.refresh();
      cc_pos.refresh(); cc_vel.refresh();
      //System.out.println("FX Position: " + fx_pos + " FX Vel: " + fx_vel);
      //System.out.println("CC Position: " + cc_pos + " CC Vel: " + cc_vel);
      //System.out.println("");
    }
  }


  public void armpositionIntake(){
    m_fx.setControl(m_dutyPosition.withPosition(kIntakePosition));
  }
  public void armpositionamp(){
    m_fx.setControl(m_dutyPosition.withPosition(kampPosition));
  }
  public void armpositionTrapPrep(){
    m_fx.setControl(m_dutyPosition.withPosition(kTrapPrepPosition));
  }
  public void armpositionTrapClimb(){
    m_fx.setControl(m_dutyPosition.withPosition(kTrapclimbPosition));
  }
public void armclearfault(){
 /* Clear sticky faults */  
  m_fx.clearStickyFaults();
}

public void  armmanual(){
  //m_fx.setControl(m_dutyCycleControl.withOutput(m_operatorController.getLeftY()));

}

public void setarm(double speed){
}

public void  armfoward(){
m_fx.setControl(m_dutyCycleControl.withOutput(.5));
}

public void armreverese(){
  m_fx.setControl(m_dutyCycleControl.withOutput(-.5));

}

public void stop(){
  m_fx.setControl(m_dutyCycleControl.withOutput(0));
}

// creates a value for shuffle board
public double getposition(){
return m_cc.getAbsolutePosition().getValue();}


public void createDashboards() {
  ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  armTab.addNumber("Arm Position", this::getposition)
  .withSize(1,1)
  .withPosition(0,0);
}


}
