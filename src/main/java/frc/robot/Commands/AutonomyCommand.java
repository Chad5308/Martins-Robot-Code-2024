package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class AutonomyCommand extends Command{
    
    public DigitalInput shooterSwitch;
    public DigitalInput intakeSwitch;

    public boolean controlMode = true; //True == Autonomous aiming, False == manuel aiming

    public ShooterCommand c_Shooter;
    public IntakeCommand c_Intake;


    public AutonomyCommand(ShooterCommand c_Shooter, IntakeCommand c_Intake){
        shooterSwitch = new DigitalInput(Constants.ShooterConstants.shooterSwitchID);
        intakeSwitch = new DigitalInput(Constants.IntakeConstants.intakeSwitchID);
        this.c_Shooter = c_Shooter;
        this.c_Intake = c_Intake;
    }
 
    public boolean getShooter(){
        return shooterSwitch.get();
    }

    public boolean getIntake(){
        return intakeSwitch.get();
    }

    public boolean getMode(){
        return controlMode;
    }

    public Command switchMode(){
        return Commands.runOnce(()-> {
        controlMode = !controlMode;
        });
    }

 


    @Override
    public void execute(){
        
            
        //TODO Take this out
        // c_Shooter.setHome().onlyWhile(() -> !c_Shooter.prepareShot().isScheduled() && controlMode == true);
        // c_Shooter.feed().until(this::getShooter).onlyWhile(() -> controlMode == true && c_Intake.deploy().isScheduled()).andThen(c_Shooter.stopBreach());
        // c_Intake.home().onlyWhile(() -> controlMode == true && !c_Intake.deploy().isScheduled() && !getIntake());

    }

}
