// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LauncherConstants.kIntakeKickerSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANLauncher;
public class endEarlyCommand extends Command {
  /** Creates a new endEarlyCommand. */
  CANLauncher m_launcher;
  public endEarlyCommand(CANLauncher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_launcher = launcher;
    
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {             
    if(CANLauncher.hasnote == true) {
    this.isFinished();
    //intakeAutCommand().isFinished();
    // stop();
    //intakeAutCommand().end(true);
    System.out.println("true");
  }
  else  {
   
    m_launcher.setFeedWheel(.38);
    m_launcher.setKickerWheel(kIntakeKickerSpeed);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    m_launcher.setFeedWheel(0);
    m_launcher.setKickerWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(CANLauncher.hasnote == true) {
    return true;
  }
    return false;

  }
}
