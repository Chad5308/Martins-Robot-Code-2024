package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{
    public ShooterSubsystem s_Shooter;
    public LimelightSubsystem s_Limelight;
    public boolean autoAim = false;
    
    public ShooterCommand(ShooterSubsystem s_Shooter, LimelightSubsystem s_Limelight){
        this.s_Shooter = s_Shooter;
        this.s_Limelight = s_Limelight;
        addRequirements(s_Shooter, s_Limelight);
    }

    @Override
    public void initialize(){
        s_Shooter.home();
    }

    @Override
    public void execute(){
        if(autoAim){
            s_Shooter.setPosition(s_Limelight.autoAngle());
        }
    }
   
    public void closeSpeaker(){
        s_Shooter.setDesiredVelocities(60, 60);
        s_Shooter.setPosition(Constants.ShooterConstants.closeSpeakerAngle);
    }
    
    public void podiumShot(){
        s_Shooter.setDesiredVelocities(75, 75);
        s_Shooter.setPosition(Constants.ShooterConstants.podiumSpeakerAngle);
    }

    public void stopBreach(){
        s_Shooter.stopFeed();
    }
   
    public void setHome(){
        s_Shooter.setDesiredVelocities(0, 0);
        s_Shooter.home();
    }


}
