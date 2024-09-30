// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.AutoCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.DriveFiles.DriveCommand;
import frc.robot.DriveFiles.SwerveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LightSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);

  public static Robot robot = new Robot();
  public ShooterSubsystem s_Shooter = new ShooterSubsystem();
  public IntakeSubsystem s_Intake = new IntakeSubsystem();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot);
  public LightSubsystem lights = new LightSubsystem(robot, s_Swerve);
  public LimelightSubsystem s_Limelight = new LimelightSubsystem(s_Swerve,s_Shooter);


  public DriveCommand c_Drive = new DriveCommand(s_Swerve, opController);
  public ShooterCommand c_Shooter = new ShooterCommand(s_Shooter, s_Limelight);
  public IntakeCommand c_Intake = new IntakeCommand(s_Intake);
  public AutoCommand c_AutoCommand = new AutoCommand(c_Drive, s_Swerve, s_Limelight, c_Intake, c_Shooter);
  // public AutonomyCommand c_Detection = new AutonomyCommand(c_Shooter, c_Intake);
  private SendableChooser<Command> autoChooser;


 
public RobotContainer() {
    s_Swerve.setDefaultCommand(c_Drive);
    s_Shooter.setDefaultCommand(c_Shooter);
    s_Intake.setDefaultCommand(c_Intake);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);   

    // configureAutos();
  }


  // public Command ampSideLeave()
  // {
  //   return new PathPlannerAuto("Amp Side Leave");
  // }
  // public Command midLeave()
  // {
  //   return new PathPlannerAuto("Mid Leave");
  // }
  // public Command sourceSideLeave()
  // {
  //   return new PathPlannerAuto("Source Side Leave");
  // }
  // public Command doNothing()
  // {
  //   return new PathPlannerAuto("Do Nothing");
  // }
  // public Command testRotation()
  // {
  //   return new PathPlannerAuto("Test");
  // }


  // public void configureAutos(){
  //   autoChooser.addOption("Mid Leave", midLeave());
  //   autoChooser.addOption("Source Side Leave", sourceSideLeave());
  //   autoChooser.addOption("Amp Side Leave", ampSideLeave());
  //   autoChooser.addOption("Rotation Test", testRotation());
  // }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  public void teleopBindings()
  {
    //Drive Controls
    opController.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button
   
 

    //Intake Controls
    opController.rightBumper().whileTrue(c_Intake.deploy());
    opController.rightBumper().whileFalse(c_Intake.home());

    opController.x().whileTrue(c_Intake.reverseIntake());
    opController.leftTrigger().whileFalse(c_Intake.intakeStop());
    opController.leftTrigger().whileTrue(c_Intake.intake());
    opController.rightTrigger().whileTrue(c_Shooter.lowShoot().alongWith(c_Intake.shootPosition()));
    opController.rightTrigger().whileFalse(c_Shooter.home());

    opController.b().whileTrue(c_Shooter.closeSpeaker());
    opController.b().whileFalse(c_Shooter.home());

    
    
    
    //Shooter Controls
    // opController.rightTrigger().whileTrue(c_Shooter.closeSpeaker().alongWith(c_Intake.shootPosition()));
    // opController.rightTrigger().whileFalse(c_Shooter.home().alongWith(c_Intake.home()));
    // opController.rightBumper().whileTrue(c_Shooter.podiumShot().alongWith(c_Intake.shootPosition()));
    // opController.rightBumper().whileFalse(c_Shooter.home().alongWith(c_Intake.home()));
    
    opController.leftBumper().whileTrue(c_Shooter.feed());
    opController.leftBumper().whileFalse(c_Shooter.feedStop());
    opController.a().whileTrue(c_Shooter.reverseFeed());
    opController.a().whileFalse(c_Shooter.feedStop());


    // opController.leftStick().whileTrue(c_Shooter.prepareShot());
    // opController.x().onTrue(c_Detection.switchMode());
    // opController.y().onTrue(c_Shooter.podiumShot().unless(c_Detection::getMode));
    // opController.b().onTrue(c_Shooter.closeSpeaker().unless(c_Detection::getMode));
    // opController.a().onTrue(c_Shooter.setHome().unless(c_Detection::getMode).alongWith(c_Shooter.feedStop().unless(c_Detection::getMode)));

  }


  public void testBindings()
  {
    opController.rightTrigger().whileTrue(c_Shooter.test());//  Right trigger
    opController.rightTrigger().whileFalse(c_Shooter.testStop());
    opController.rightBumper().whileTrue(c_Shooter.feed());//   Right Bumper
    opController.rightBumper().whileFalse(c_Shooter.feedStop());

    // opController.povRight().onTrue(c_Shooter.setHome());//      right
    opController.povRight().onTrue(c_Shooter.positionTest());
    // opController.povDown().whileFalse(c_Shooter.stopPitch());
    // opController.povUp().whileFalse(c_Shooter.stopPitch());
    
    // opController.b().onTrue(c_Intake.home());//           b.
    // opController.x().whileTrue(c_Intake.deploy());//      x.

    opController.x().onTrue(c_Intake.pitchTest());//     x.
    opController.b().whileTrue(c_Intake.intakeTest());//    b.
    opController.b().whileFalse(c_Intake.intakeStop());//   b.
  }
}
