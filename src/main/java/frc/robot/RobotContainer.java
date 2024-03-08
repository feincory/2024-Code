
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;

//import frc.robot.commands.Autos;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
//import frc.robot.subsystems.PWMDrivetrain;
//import frc.robot.subsystems.PWMLauncher;
import frc.robot.generated.TunerConstants;
//import frc.robot.Constants.OperatorConstants;

//import frc.robot.commands.Autos;
//import frc.robot.commands.LaunchNote;
//import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.CANdleSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //sendable
  private final SendableChooser<Command> autoChooser;
  
  
  //swerve section
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.75 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final CommandXboxController m_operatorController =
     new CommandXboxController(OperatorConstants.kOperatorControllerPort); // My joystick
  public final CommandJoystick m_drivercontroller = 
    new CommandJoystick(OperatorConstants.kDriverControllerPort); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.06) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.RobotCentric limelightrotate = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private final Command runAuto;
  //private Command runAuto = drivetrain.getAutoPath("SimplePath");
  private final Telemetry logger = new Telemetry(MaxSpeed);


  // The robot's subsystems are defined here.
  // private final PWMDrivetrain m_drivetrain = new PWMDrivetrain();
  //private final CANDrivetrain m_drivetrain = new CANDrivetrain();
  // private final PWMLauncher m_launcher = new PWMLauncher();

  public final CANLauncher m_launcher = new CANLauncher();
  public final arm m_arm = new arm();
  public final climber m_climber = new climber();
  public final CANdleSystem m_led = new CANdleSystem(m_operatorController);
  public double kLLpcontroller;
  /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    NamedCommands.registerCommand("runintake", m_launcher.intakeAutCommand());
    NamedCommands.registerCommand("armintakepos", new RunCommand(m_arm::armpositionIntake));
    NamedCommands.registerCommand("preparelaunch", m_launcher.autoLaunch());
    NamedCommands.registerCommand("launchnote", m_launcher.setlaunchCommand());
    NamedCommands.registerCommand("autoaim", new RunCommand(m_arm::armAutoRotateCommand));

     runAuto = drivetrain.getAutoPath("Blue Center");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    kLLpcontroller = .075;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be accessed via the
   * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  private void configureBindings() {
    // Set the default command for the drivetrain to drive using the joysticks
    //swerve button bindings
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(m_drivercontroller.getRawAxis(1) * MaxSpeed) // Drive forward with
                                                                                          // negative Y (forward)
            .withVelocityY(-m_drivercontroller.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_drivercontroller.getRawAxis(3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    m_drivercontroller.button(12).whileTrue(drivetrain.applyRequest(() -> brake));    

    //m_drivercontroller.button(16).whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_drivercontroller.getRawAxis(0), m_drivercontroller.getRawAxis(1)))));
    
        
      // reset the field-centric heading on reset button of flight controller
    m_drivercontroller.button(14).whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

 
    drivetrain.registerTelemetry(logger::telemeterize);


    m_drivercontroller.button(22).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-.5).withVelocityY(0)));
    m_drivercontroller.button(21).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    m_drivercontroller.button(19).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
    m_drivercontroller.button(20).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));  
    
    
    //limelight autoline up
    m_operatorController.a().whileTrue(drivetrain.applyRequest(() -> 
      drive.withVelocityX(m_drivercontroller.getRawAxis(1) * MaxSpeed) // Drive forward with
      .withVelocityY(-m_drivercontroller.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate((-LimelightHelpers.getTX(null)+2)*kLLpcontroller)))
      .whileTrue(new RunCommand(m_arm::armAutoRotateCommand))
      .onFalse(new InstantCommand(m_arm::stop))
      .whileTrue(new PrepareLaunch(m_launcher)
      .handleInterrupt(() -> m_launcher.stop())); 

    //climber auto line up
    // m_drivercontroller.button(25).whileTrue(drivetrain.applyRequest(() -> 
    //   drive.withVelocityX((-.7 - LimelightHelpers.getTX(null))*0) // Drive forward with
    //   .withVelocityY((0-LimelightHelpers.getTY(null))*0) //
    //   .withRotationalRate(0-drivetrain.getPigeon2().getYaw().getValueAsDouble() * .10 ))); //

    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    // m_operatorController.rightBumper().whileTrue(
    //         new PrepareLaunch(m_launcher)
    //             .withTimeout(LauncherConstants.kLauncherDelay)
    //             .andThen(new LaunchNote(m_launcher))
    //             .handleInterrupt(() -> m_launcher.stop()));

    m_operatorController.rightBumper().whileTrue(
        new PrepareLaunch(m_launcher).handleInterrupt(() -> m_launcher.stop()));
    
    m_operatorController.leftStick().whileTrue(
        new LaunchNote(m_launcher).handleInterrupt(() -> m_launcher.stop()));
                
    // Set up a binding to run the intake command while the operator is pressing and holding the
    //left Bumper
    m_operatorController.leftBumper().whileTrue(m_launcher.getIntakeCommand())
                                      .onFalse(new RunCommand(m_launcher::noteMoveForshot)
                                      .withTimeout(.08)//was .05
                                      .andThen(new InstantCommand(m_launcher::stop)));
                         
    //m_operatorController.leftBumper().onTrue(m_launcher.intakeAutCommand());                                  



                                      
    m_operatorController.y().whileTrue(m_launcher.getReverseNoteCommand().andThen(m_launcher::noteMoveForshot));
    // m_operatorController.x().onTrue(new InstantCommand(m_arm::armfoward))
    //                       .onFalse(new InstantCommand(m_arm::stop));
    // m_operatorController.b().onTrue(new InstantCommand(m_arm::armreverese))
    //                       .onFalse(new InstantCommand(m_arm::stop));
      
   //climber
   m_drivercontroller.button(8).onTrue(new InstantCommand(m_climber::climberReleaseable));
   m_drivercontroller.button(8).onFalse(new InstantCommand(m_climber::climberNotReleaseable));
    
   
   
   m_operatorController.b().onTrue(
      new InstantCommand(m_climber::climberencoderreset)
      .andThen(new InstantCommand(m_climber::climbReleaseCommand))
      .andThen(new InstantCommand(m_arm::armpositionTrapPrep))
      .andThen(new InstantCommand(m_launcher::noteMoveForAmp))
      .withTimeout(1)
      .andThen(m_launcher::stop))
      
      .onFalse(new InstantCommand(m_climber::stop));

    m_operatorController.x().onTrue(
      new InstantCommand(m_climber::climbwinchbottom))
      .onFalse(new InstantCommand(m_climber::stop));      

    m_operatorController.x().onTrue(
      new InstantCommand(m_arm::armpositionTrapClimb)); 

    m_operatorController.povUp().onTrue(
      new InstantCommand(m_arm::defenciveshot));

     // .andThen(new RunCommand(m_arm::armpositionTrapPrep))
     // .andThen(new RunCommand(m_climber::climbNotSafe))                         
  
     m_operatorController.leftTrigger(.8).onTrue(new InstantCommand(m_climber::climbupmanual))
                             .onFalse(new InstantCommand(m_climber::stop));
    m_operatorController.rightTrigger(.8).onTrue(new InstantCommand(m_climber::climbdownmanual))
                             .onFalse(new InstantCommand(m_climber::stop));
    //servo
   // m_drivercontroller.button(6).onTrue(new InstantCommand(m_climber::servoPreDeploy));
   // m_drivercontroller.button(7).onTrue(new InstantCommand(m_climber::servoPostDeploy));
    m_drivercontroller.button(4).onTrue(new InstantCommand())
                                       .onFalse(new InstantCommand(m_arm::stop));    
  

  
   m_operatorController.povDown().onTrue(new InstantCommand(m_arm::armpositionIntake));
                                        //.andThen(new RunCommand( m_launcher::noteMoveForAmp)));
                                        //.withTimeout(.25)
                                        //.andThen(m_launcher::stop));
  m_operatorController.povRight().onTrue(new InstantCommand(m_arm::armpositionamp)
                                        .andThen(new RunCommand( m_launcher::noteMoveForAmp))
                                        .withTimeout(1)
                                        .andThen(m_launcher::stop));    
  // m_operatorController.rightBumper()/* .onTrue(new InstantCommand(m_arm::StageShot))*/
  //                                       .onFalse(new RunCommand(m_launcher::noteMoveForshot)
  //                                     .withTimeout(.08)//was .05
  //                                     .andThen(new InstantCommand(m_launcher::stop)));
  m_operatorController.povLeft().onTrue(new InstantCommand(m_arm::StageShot));//subwoofer/speaker shot
  
 m_operatorController.back().onTrue(m_arm.armcommpDown());
  m_operatorController.start().onTrue(m_arm.armcommpUp());

  // m_operatorController.back().onTrue(new InstantCommand(m_arm::armclearfault));


  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return runAuto;
    
   // return new PathPlannerAuto("New Auto");
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_drivetrain);
  }

  // private void buildDashboard(){
  //   buildarmTab();
    

  // }

  // private void buildarmTab(){

  //   ShuffleboardTab armTab = Shuffleboard.getTab("arm");

  // }
}
