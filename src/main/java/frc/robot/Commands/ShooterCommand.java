package frc.robot.Commands;

import com.revrobotics.CANSparkBase.ControlType;

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

    @Override
    public void initialize(){
        // s_Shooter.home();
    }

    @Override
    public void execute(){}

   //Manuel Commands
    public Command closeSpeaker(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(60, 60);
            s_Shooter.setPosition(Constants.ShooterConstants.closeSpeakerAngle);
        });
    }
    
    public Command podiumShot(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(75, 75);
            s_Shooter.setPosition(Constants.ShooterConstants.podiumSpeakerAngle);
        });
    }

    public Command stopBreach(){
        return Commands.runOnce(()->{
            s_Shooter.setBreach(0);
        });
    }


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

    public Command pitchUp()
    {
        return Commands.run(() -> 
        {
            s_Shooter.pitchMotor.set(0.085);
        });
    }

    public Command pitchDown()
    {
        return Commands.run(() ->
        {
            s_Shooter.pitchMotor.set(-0.085);
        });
    }

    public Command stopPitch()
    {
        return Commands.run(()->
        {
            s_Shooter.pitchMotor.set(0);
        });
    }

    public Command positionTest()
    {
        return Commands.run(()->
        {
            s_Shooter.setPosition(25);
        });
    }



    // Autonomy Commands

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
   
    public Command setHome(){
        return Commands.runOnce(()->{
            s_Shooter.setDesiredVelocities(0, 0);
            s_Shooter.home();
        });
    }

    public Command launch(){
        return Commands.runOnce(()->{
            s_Shooter.setBreach(1);
        });
    }

    public Command prepareShot(){
        return Commands.run(()->{
            s_Shooter.autoSpeeds();
            s_Shooter.setPosition(s_Limelight.autoAngle());
        });
    }







}
