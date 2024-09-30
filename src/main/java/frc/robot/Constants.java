// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(17.5);
      // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(17.5);
      // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //back left
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //back right


    //start front front left to front right to back right and all drives then all steers then all absolutes
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kBackLeftTurningMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 7;
    public static final int kBackRightDriveMotorPort = 8;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
    public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    //TODO Test and input all module offsets
    public static final double kFLDegrees = 5.09765625;
    public static final double kFRDegrees = 151.69921875;
    public static final double kBRDegrees = 132.5390625;
    public static final double kBLDegrees = -109.86328125;


    //TODO Invert any motor to match controller output
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.60248; //6.949 for Swerve X, 4.60248 for sds
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =kPhysicalMaxSpeedMetersPerSecond/(kTrackWidth/2);

    //For limiting speed while driving
    public static final double kTeleDriveSpeedLimiter = 1; //TOO LIMIT SPEED CHANGE THIS
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * kTeleDriveSpeedLimiter;  
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * kTeleDriveSpeedLimiter;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.35;
  }
  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.75 / 1; //4.59 for Swerve X, 6.75 for sds
    public static final double kTurningMotorGearRatio = 12.8 / 1; //13.3714 for Swerve X, 12.8 for sds
    public static final double kDriveEncoderRot2Meter = 1/23.58; //Not sure try 1/16.0344, 1/23.58 for sds
    
    public static final double kTurningConversionFactor2Deg =  28.25;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2DegPerSec = kTurningConversionFactor2Deg / 60;

    public static final double kPTurning = 0.0075;
    public static final double kITurning = 0;
    public static final double kDTurning = 0.75;

    public static final double moduleRadius = Units.inchesToMeters(Constants.DriveConstants.kTrackWidth/2); //measured from center of robot to furthest module.
  }

  public static final class OIConstants {
    public static final int kOPControllerPort = 0;
    public static final double kDeadband = 0.09;
    public static final int kLeftStickPort = 1;
    public static final int kRightStickPort = 2;
  }
 

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeedRadiansPerSecond =  DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    public static final double kMaxAngularAccelerationUnitsPerSecond = DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    public static  double kPTranslation = 5;
    public static  double kITranslation = 0;
    public static  double kDTranslation = 0;

    public static final double kPTheta = 5;
    public static final double kITheta = 0;
    public static final double kDTheta = 0;


    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationUnitsPerSecond);
    public static final TrapezoidProfile.Constraints kLinearConstraints = 
            new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared
            );
  }

  public static final class AprilTagIds{
    public static final int blueSourceRight = 1;
    public static final int blueSourceLeft = 2;
    public static final int redSpeakerRight = 3;
    public static final int redSpeakerLeft = 3;
    public static final int redAmp = 5;
    public static final int blueAmp = 6;
    public static final int blueSpeakerRight = 7;
    public static final int blueSpeakerLeft = 8;
    public static final int redSourceRight = 9;
    public static final int redSourceLeft = 10;
    public static final int redStageSource = 11;
    public static final int redStageAmp = 12;
    public static final int redStageCenter = 13;
    public static final int blueStageCenter = 14;
    public static final int blueStageAmp = 15;
    public static final int blueStageSource = 16;
  }

  public static final class limelightConstants{
    public static final double thetakP = 3.0;
    public static final double thetakI = 0;
    public static final double thetakD = 0;

    public static final double linearkP = 0.4;
    public static final double linearkI = 0;
    public static final double linearkD = 0;
    public static final double angleOffset = 35; //degrees TODO Find this value
  }


  public static final class IntakeConstants{
    public static final int intakeMotorID = 20;
    public static final int pitchMotorID = 21;
    public static final int pitchABSEncoder = 7;
    public static final int intakeSwitchID = 1;

    public static final double gearRatio = 0.05;
    public static final boolean pitchMotorReversed = false;
    public static final boolean intakeMotorReversed = true;

    public static final double kP_pitch = 0.003;
    public static final double kI_pitch = 0.000005;
    public static final double kD_pitch = 0.00075;

    public static final double kS_intake = 0.25;
    public static final double kV_intake = 0.075;
    public static final double kA_intake = 0.0025;
    public static final double kP_intake = 0.00001;
    public static final double kI_intake = 0;
    public static final double kD_intake = 0;

  }

  public static final class ShooterConstants{
  //CAN Bus Numbers
    public static final int topMotorID = 30;
    public static final int bottomMotorID = 31;
    public static final int breachMotorID = 32;
    public static final int PitchEncoderID = 33;
    public static final int pitchABSEncoder = 8;
    public static final int shooterSwitchID = 0;

    public static final double gearRatio = 0.05;


    public static final double ramp_rate = 0.1;


    //PID Values
    public static final double kS_TopShooter = 0.4;
    public static final double kV_TopShooter = 0.12;
    public static final double kA_TopShooter = 0.04;
    public static final double kP_TopShooter = 0.000025;
    public static final double kI_TopShooter = 0;
    public static final double kD_TopShooter = 0.01;

    public static final double kS_BottomShooter = 0.4;
    public static final double kV_BottomShooter = 0.12;
    public static final double kA_BottomShooter = 0.1;
    public static final double kP_BottomShooter = 1.0;
    public static final double kI_BottomShooter = 0;
    public static final double kD_BottomShooter = 0.01;

    public static final double kP_pitch = 0.009;
    public static final double kI_pitch = 0.000025;
    public static final double kD_pitch = 0.05;

    public static final double kP_breach = 0.01;
    public static final double kI_breach = 0;
    public static final double kD_breach = 0;

    //Encoder values
    public static final boolean topMotorReversed = false;
    public static final boolean bottomMotorReversed = false;
    public static final boolean breachReversed = false;
    public static final boolean pitchReversed = true;



     //Known Angles TODO Find these values after testing
     public static final double closeSpeakerAngle = 50;
     public static final double podiumSpeakerAngle = 30;
}




}

