package frc.robot.Subsystems;

import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DriveFiles.SwerveSubsystem;

public class LimelightSubsystem extends SubsystemBase{
    
public SwerveSubsystem s_swerve;
public ShooterSubsystem s_Shooter;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
// public double targetHeight = 1.450975;
// public double cameraHeight = 1.2065;
public double xAng, yAng, hasTargets, zSpeed, xSpeed, turningSpeed, targetID;
public double correctionX, correctionZ, correctionT;
public double distanceX, distanceZ;
public double yAngToDegrees;
public double[] localizedPose;
public double[] botPose_targetSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController ZPIDController;
public ProfiledPIDController XPIDController;
public boolean autoAlign = false;

    public LimelightSubsystem(SwerveSubsystem s_swerve, ShooterSubsystem s_Shooter){
        this.s_swerve = s_swerve;
        this.s_Shooter = s_Shooter;

// NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);



        networkTables = NetworkTableInstance.getDefault().getTable("limelight");

        pipeline = networkTables.getIntegerTopic("limelight.getpipe").subscribe(0);
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        thetaPIDController = new ProfiledPIDController(Constants.limelightConstants.thetakP, Constants.limelightConstants.thetakI, Constants.limelightConstants.thetakD, Constants.AutoConstants.kThetaControllerConstraints);
        ZPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
        XPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
    }

    public Optional<Pose2d> getPoseFromAprilTags() {
        double[] botpose = localizedPose;
        if(botpose.length < 7 || targetID == -1) return Optional.empty();
        return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    }
    public void setThetaPID(){
        thetaPIDController.setGoal(0);
        thetaPIDController.setTolerance(Math.toRadians(10));
        }
    public void setLinearPID(){
        ZPIDController.setGoal(0.75); //inches
        ZPIDController.setTolerance(0.25); //meters

        XPIDController.setGoal(0);
        XPIDController.setTolerance(0.25);
    }
  
    public double autoAngle(){
        if (targetID == Constants.AprilTagIds.redSpeakerLeft || targetID == Constants.AprilTagIds.blueSpeakerLeft) {

            //TODO Find limelight mount angle
            double targetHeight = 57.125; //Inches From ground to middle of tag (See April Tag Manuel for dimensions)
            double cameraHeight = 1; //Find Camera height (ground to lens)
            double distance2Target = ((targetHeight-cameraHeight)/ (Math.tan(yAng)));//Inches

            double targetHeight2Hood = 23.541; //Inches
            double finalAngle = Math.toDegrees(Math.atan((targetHeight2Hood+targetHeight)/distance2Target));

            return finalAngle;
        }else {
            return s_Shooter.getPosition();
        }
    }

    // public double autoAlign(){
    //     if(autoAlign == true && (targetID == Constants.AprilTagIds.redSpeakerLeft || targetID == Constants.AprilTagIds.blueSpeakerLeft) && !thetaPIDController.atSetpoint()){
    //         return (thetaPIDController.getSetpoint().velocity + correctionT);
    //     }else {
    //         return 0.0;
    //     }
    // }


@Override
public void periodic(){
    // if (s_swerve.allianceCheck() == true) {
    //             localizedPose = networkTables.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    // } else {
    //         localizedPose = networkTables.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    //     }

botPose_targetSpace = networkTables.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
localizedPose = networkTables.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
xAng = Math.toRadians(networkTables.getEntry("tx").getDouble(0));
yAng = Math.toRadians(networkTables.getEntry("ty").getDouble(0)) + Math.toRadians(Constants.limelightConstants.angleOffset);
yAngToDegrees = Math.toDegrees(yAng);
hasTargets = networkTables.getEntry("tv").getDouble(0);
targetID = networkTables.getEntry("tid").getDouble(0);
// distanceX = ((targetHeight-cameraHeight) / (Math.tan(yAng)));//inches
// distanceZ = distanceX * Math.tan(xAng);//inches
// distanceX = botPose_targetSpace[0];
// distanceZ = Math.abs(botPose_targetSpace[2]);
// correctionX = XPIDController.calculate(distanceX);//meters
// correctionZ = ZPIDController.calculate(distanceZ);//meters
correctionT = thetaPIDController.calculate(xAng);//radians
// SmartDashboard.putNumber("Auto Angle", autoAngle()); **



// SmartDashboard.putNumber("Distance X", distanceX);
// SmartDashboard.putNumber("Distance Z", distanceZ);
// SmartDashboard.putNumber("Turning Speed", turningSpeed);
// SmartDashboard.putNumber("X Speed", zSpeed);
// SmartDashboard.putNumber("Y Speed", xSpeed);
// SmartDashboard.putNumber("TX Value", xAng); **
// SmartDashboard.putNumber("TY Value", yAng);
// SmartDashboard.putNumber("TY degrees", Math.toDegrees(yAng));
// SmartDashboard.putBoolean("Is command running", autoAlign);


// SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
// SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
// SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
// // if(!XPIDController.atSetpoint() && ZPIDController.atSetpoint()){
// //     xSpeed = XPIDController.getSetpoint().velocity + correctionX;
// //     zSpeed = ZPIDController.getSetpoint().velocity + correctionZ;
// //     turningSpeed = 0;
// // }
