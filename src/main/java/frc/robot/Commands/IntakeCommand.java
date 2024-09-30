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
        if(s_Intake.getAbsolutePosition()>100)
        {
            s_Intake.pitchMotorEncoder.setPosition(s_Intake.getAbsolutePosition()-129);
        }else{
            s_Intake.pitchMotorEncoder.setPosition(s_Intake.getAbsolutePosition());
        }
    }




    public Command home(){
        return Commands.runOnce(()->{
            s_Intake.setPosition(120);
            s_Intake.setIntakeSpeed(0);
        });
    }

    public Command deploy(){
        return Commands.runOnce(()->{
            s_Intake.setPosition(0);
        });
    }

    public Command shootPosition()
    {
        return Commands.runOnce(()->{
            s_Intake.setPosition(60);
        });
    }

    public Command ampPosition()
    {
        return Commands.runOnce(()->{
            s_Intake.setPosition(90);
        });
    }

    public Command reverseIntake()
    {
         return Commands.run(()->{
            s_Intake.setIntakeSpeed(17.5);
        });
    }

    public Command intake()
    {
        return Commands.run(()->{
            s_Intake.setIntakeSpeed(-17.5);
        });
    }

    public Command intakeStop()
    {
        return Commands.run(()->
        {
            s_Intake.setIntakeSpeed(0);//RPS
        });
    }
    // public Command ampRoutine()
    // {
    //     return Commands.run(()->{
    //         ampPosition();
    //         reverseIntake();
    //     });
    // }

   

    //Test Commands

    public Command pitchTest()
    {
        return Commands.run(()->
        {
            s_Intake.setPosition(100);
        });
    }

    public Command intakeTest()
    {
        return Commands.run(()->
        {
            s_Intake.setIntakeSpeed(20);//RPS
        });
    }



    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }



}
