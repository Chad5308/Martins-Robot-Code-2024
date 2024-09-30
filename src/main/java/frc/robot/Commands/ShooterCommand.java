package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{
    public ShooterSubsystem s_Shooter;
    public LimelightSubsystem s_Limelight;
    
    public ShooterCommand(ShooterSubsystem s_Shooter, LimelightSubsystem s_Limelight){
        this.s_Shooter = s_Shooter;
        this.s_Limelight = s_Limelight;
        addRequirements(s_Shooter, s_Limelight);
    }

  

   //Manuel Commands
    public Command closeSpeaker(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(100, 100);
            s_Shooter.setPosition(Constants.ShooterConstants.closeSpeakerAngle);
        });
    }
    
    public Command podiumShot(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(100, 100);
            s_Shooter.setPosition(Constants.ShooterConstants.podiumSpeakerAngle);
        });
    }

    public Command lowShoot()
    {
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(100, 100);
            s_Shooter.setPosition(10);
        });
    }
    
    public Command home(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(0, 0);
            s_Shooter.setPosition(0);
        });
    }
    
    public Command feed(){
        return Commands.run(()->{
            s_Shooter.setBreach(1);
        });
    }

    public Command feedStop(){
        return Commands.run(()->{
            s_Shooter.setBreach(0);
        });
    }

    public Command reverseFeed()
    {
        return Commands.run(()->{
            s_Shooter.setBreach(-0.5);
        });
    }
    
    // public Command prepareShot(){
        //     return Commands.run(()->{
            //         s_Shooter.autoSpeeds();
            //         s_Shooter.setPosition(s_Limelight.autoAngle());
            //     });
            // }
            
            
            
        //Test Commands
    
    public Command test(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(100,100);
        });
    }
    public Command testStop(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(0, 0);
        });
    }

    public Command positionTest()
    {
        return Commands.runOnce(()->
        {
            s_Shooter.setPosition(60);
        });
    }




    
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }





}
