package frc.robot.Subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    public CANSparkMax pitchMotor;
    public CANSparkMax intakeMotor;

    public SparkPIDController pitchMotorPID;
    public SparkPIDController intakeMotorPID;

    public AbsoluteEncoder pitchMotorEncoder;
    public RelativeEncoder intakeMotorEncoder;

    public boolean pitchReversed;
    public boolean intakeReversed;

    public double desiredPos; //degrees
    public double posTolerance = 1; //degrees


    public IntakeSubsystem(){
        //1 for pitch Neo, 1 Neo for driving intake rollers, 1 bore encoder
        pitchMotor = new CANSparkMax(Constants.IntakeConstants.pitchMotorID, MotorType.kBrushless);
        pitchMotorEncoder = pitchMotor.getAbsoluteEncoder();
        pitchMotorPID = pitchMotor.getPIDController();
        configure();
    }


    public void configure(){
        //Pitch Config
        pitchMotor.setInverted(Constants.IntakeConstants.pitchMotorReversed);
        pitchMotor.setIdleMode(IdleMode.kBrake);

        pitchMotorPID.setP(Constants.IntakeConstants.kP_pitch);
        pitchMotorPID.setI(Constants.IntakeConstants.kI_pitch);
        pitchMotorPID.setD(Constants.IntakeConstants.kD_pitch);

        pitchMotorEncoder.setPositionConversionFactor(360); //TODO Multiply by gear ratio of pitch motor when found
        pitchMotor.setOpenLoopRampRate(5);
        pitchMotorPID.setSmartMotionMaxVelocity(5, 0);
        pitchMotorEncoder.setZeroOffset(0); //TODO Find this once built

        //Intake Config
        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorReversed);
        intakeMotor.setIdleMode(IdleMode.kCoast);

        intakeMotorPID.setP(Constants.IntakeConstants.kP_intake);
        intakeMotorPID.setI(Constants.IntakeConstants.kI_intake);
        intakeMotorPID.setD(Constants.IntakeConstants.kD_intake);
        intakeMotor.setOpenLoopRampRate(1);
    }


    //Pitch Methods
    public void setPosition(double degrees){
        pitchMotorPID.setReference(degrees, ControlType.kPosition);
    }

    public void gravity(){
        pitchMotorPID.setP((0.001 * Math.sin(Math.toRadians(pitchMotorEncoder.getPosition()))) + Constants.IntakeConstants.kP_pitch);//TODO Make sure the gravity constant is working
    }

    public double getPosition(){
        return pitchMotorEncoder.getPosition();
    }

    //Intake Methods
    public void setIntakeSpeed(double speed){
       intakeMotor.set(speed);
    }


    @Override
    public void periodic(){
        getPosition();
        SmartDashboard.putNumber("IntakePosition", getPosition());
    }
    
}
