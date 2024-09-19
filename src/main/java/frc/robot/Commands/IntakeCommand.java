package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    

    public IntakeSubsystem s_Intake;

    public IntakeCommand(IntakeSubsystem s_Intake){
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize(){
        // home();
    }



    public Command home(){
        return Commands.runOnce(()->{
            s_Intake.setPosition(0);
            s_Intake.setIntakeSpeed(0);
        });
    }

    public Command deploy(){
        return Commands.runOnce(()->{
            s_Intake.setPosition(125);//TODO Check Value
            s_Intake.setIntakeSpeed(1);
        });
    }

    public Command pitchUp()
    {
        return Commands.run(() ->
        {
            s_Intake.setPosition(s_Intake.getPosition() - 2);
        });
    }

    public Command pitchDown()
    {
        return Commands.run(() ->
        {
            s_Intake.setPosition(s_Intake.getPosition() + 2);
        });
    }





}
