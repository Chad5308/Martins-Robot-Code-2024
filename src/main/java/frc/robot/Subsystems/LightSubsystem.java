package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LightSubsystem extends SubsystemBase{

public Spark lights;


public LightSubsystem(){
    lights = new Spark(9);
}

@Override
public void periodic(){
    lights.set(0.57);
}



}
