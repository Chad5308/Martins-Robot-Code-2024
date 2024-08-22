package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    

    public IntakeSubsystem s_Intake;

    public IntakeCommand(IntakeSubsystem s_Intake){
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }









}
