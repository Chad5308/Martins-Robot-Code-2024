// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.AutoCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.DriveFiles.DriveCommand;
import frc.robot.DriveFiles.LimelightSubsystem;
import frc.robot.DriveFiles.SwerveSubsystem;
import frc.robot.Subsystems.LightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private final CommandJoystick leftStick = new CommandJoystick(OIConstants.kLeftStickPort);
  private final CommandJoystick rightStick = new CommandJoystick(OIConstants.kRightStickPort);
  public boolean isJoystick = false;
  private SendableChooser<Command> controlType;

  public static Robot robot = new Robot();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot);
  public LimelightSubsystem s_Limelight = new LimelightSubsystem(s_Swerve);
  public DriveCommand c_Drive = new DriveCommand(s_Swerve, opController, leftStick, rightStick);
  public ShooterCommand c_Shooter = new ShooterCommand();
  public IntakeCommand c_Intake = new IntakeCommand();
  public AutoCommand c_AutoCommand = new AutoCommand(c_Drive, s_Swerve, s_Limelight);
  public LightSubsystem lights = new LightSubsystem(robot, s_Swerve);
  private SendableChooser<Command> autoChooser;


 
public RobotContainer() {
    s_Swerve.setDefaultCommand(c_Drive);
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);   

    controlType = new SendableChooser<>();
    configureAutos();
    SmartDashboard.putData("Control Chooser", controlType); 
  }



  public void configureAutos(){
    autoChooser.addOption("TestAuto", testAuto());
  }

  public Command testAuto(){
    return new PathPlannerAuto("testAuto");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  private void configureBindings() {
    //Drive Controls
    opController.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button

  
    rightStick.button(4).toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    rightStick.button(3).toggleOnTrue(s_Swerve.fieldOrientedToggle());
    rightStick.button(2).onTrue(s_Swerve.resetWheels()); //window looking button
   
 
    //test for PID
    // shootController.y().onTrue(Commands.runOnce(() -> {pitchSubsystem.setPosition(Constants.ShooterConstants.closeSpeakerAngle);}));
  }
}
