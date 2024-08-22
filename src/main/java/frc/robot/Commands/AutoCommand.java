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
    
public DriveCommand driveCommand;
public SwerveSubsystem swerveSubsystem;
public LimelightSubsystem limelightSubsystem;
public PIDController translationConstants = new PIDController(Constants.AutoConstants.kPTranslation, Constants.AutoConstants.kITranslation, Constants.AutoConstants.kDTranslation);
public PIDController rotationConstants = new PIDController(Constants.AutoConstants.kPTheta, Constants.AutoConstants.kITheta, Constants.AutoConstants.kDTheta);





    public AutoCommand(DriveCommand driveCommand, SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem){
        this.driveCommand = driveCommand;
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        // translationConstants.setTolerance(0.1);//meters
        // rotationConstants.setTolerance(10); //maybe degrees?

        AutoBuilder.configureHolonomic(
                swerveSubsystem::getPose, 
                swerveSubsystem::resetOdometry, 
                swerveSubsystem::getRobotRelativeSpeeds, 
                swerveSubsystem::driveRobotRelative, 
                autoConfig, 
                swerveSubsystem::allianceCheck,
                swerveSubsystem
                );
                
        NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> swerveSubsystem.faceAllFoward()));
    }


    public HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()), 
        Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
        Constants.ModuleConstants.moduleRadius, 
        new ReplanningConfig());

}
