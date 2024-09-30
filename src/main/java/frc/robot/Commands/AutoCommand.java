package frc.robot.Commands;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.DriveFiles.DriveCommand;
import frc.robot.DriveFiles.SwerveSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class AutoCommand {
    
public DriveCommand c_Drive;
public SwerveSubsystem s_Swerve;
public LimelightSubsystem s_Limelight;
public PIDController translationConstants = new PIDController(Constants.AutoConstants.kPTranslation, Constants.AutoConstants.kITranslation, Constants.AutoConstants.kDTranslation);
public PIDController rotationConstants = new PIDController(Constants.AutoConstants.kPTheta, Constants.AutoConstants.kITheta, Constants.AutoConstants.kDTheta);






    public AutoCommand(DriveCommand c_Drive, SwerveSubsystem s_Swerve, LimelightSubsystem s_Limelight, IntakeCommand c_Intake, ShooterCommand c_Shooter){
        this.c_Drive = c_Drive;
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        // translationConstants.setTolerance(0.1);//meters
        // rotationConstants.setTolerance(10); //maybe degrees?

        AutoBuilder.configureHolonomic(
                s_Swerve::getAutoPose, 
                s_Swerve::resetOdometry, 
                s_Swerve::getRobotRelativeSpeeds, 
                s_Swerve::driveRobotRelative, 
                autoConfig, 
                s_Swerve::allianceCheck,
                s_Swerve
                );
            
    
                
        NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> s_Swerve.faceAllFoward()));
        NamedCommands.registerCommand("Deploy", c_Intake.deploy().alongWith(c_Shooter.feed()));
        NamedCommands.registerCommand("HomeIntake", c_Intake.home());
        NamedCommands.registerCommand("ShootPositionIntake", c_Intake.shootPosition());
        NamedCommands.registerCommand("HomeShooter", c_Shooter.home());
        NamedCommands.registerCommand("BreachPrepare", c_Shooter.closeSpeaker());
        NamedCommands.registerCommand("Feed", c_Shooter.feed());
        NamedCommands.registerCommand("StopFeed", c_Shooter.feedStop());
        NamedCommands.registerCommand("Run Intake", c_Intake.intake());
        NamedCommands.registerCommand("Stop Intake", c_Intake.intakeStop());
    }


    public HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()), 
        Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
        Constants.ModuleConstants.moduleRadius, 
        new ReplanningConfig());

}
