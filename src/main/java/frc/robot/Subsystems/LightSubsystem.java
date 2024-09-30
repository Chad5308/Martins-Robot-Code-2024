package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.DriveFiles.SwerveSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LightSubsystem extends SubsystemBase{

public Spark lights;
public SwerveSubsystem s_Swerve;


public LightSubsystem(Robot robot, SwerveSubsystem s_Swerve){
    lights = new Spark(9);
    this.s_Swerve = s_Swerve;

}




@Override
public void periodic(){
    // lights.set(0.27);
    if(s_Swerve.allianceCheck()){
        lights.set(-0.25);
    }else{
        lights.set(-0.23);
    }

}



}
