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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    public CANSparkMax pitchMotor;
    public CANSparkMax intakeMotor;

    public SparkPIDController pitchMotorPID;
    public SparkPIDController intakeMotorPID;

    public DutyCycleEncoder pitchAbsoluteEncoder;
    public RelativeEncoder pitchMotorEncoder;
    public RelativeEncoder intakeMotorEncoder;

    public boolean pitchReversed;
    public boolean intakeReversed;

    public double desiredPos; //degrees
    public double posTolerance = 1; //degrees


    public IntakeSubsystem(){
        //1 for pitch Neo, 1 Neo for driving intake rollers, 1 bore encoder
        pitchMotor = new CANSparkMax(Constants.IntakeConstants.pitchMotorID, MotorType.kBrushless);
        pitchMotorEncoder = pitchMotor.getEncoder();
        pitchMotorPID = pitchMotor.getPIDController();
        pitchAbsoluteEncoder = new DutyCycleEncoder(Constants.IntakeConstants.pitchABSEncoder);
        
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotorEncoder = intakeMotor.getEncoder();
        intakeMotorPID = intakeMotor.getPIDController();

        configure();
    }


    public void configure(){
        //Pitch Config
        pitchMotor.setInverted(Constants.IntakeConstants.pitchMotorReversed);
        pitchMotor.setIdleMode(IdleMode.kBrake);

        pitchMotorPID.setP(Constants.IntakeConstants.kP_pitch);
        pitchMotorPID.setI(Constants.IntakeConstants.kI_pitch);
        pitchMotorPID.setD(Constants.IntakeConstants.kD_pitch);

        pitchMotorEncoder.setPositionConversionFactor(360 * Constants.IntakeConstants.gearRatio);
        pitchMotor.setOpenLoopRampRate(5);
        pitchMotorPID.setSmartMotionMaxVelocity(5, 0);

        pitchAbsoluteEncoder.setPositionOffset(0.5);

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

    public double getPosition(){
        double angle = pitchAbsoluteEncoder.getAbsolutePosition() - pitchAbsoluteEncoder.getPositionOffset();
        return angle*360;
    }
    
    public void gravity(){
        pitchMotorPID.setP((0.001 * Math.sin(Math.toRadians(pitchMotorEncoder.getPosition()))) + Constants.IntakeConstants.kP_pitch);//TODO Make sure the gravity constant is working
    }


    //Intake Methods
    public void setIntakeSpeed(double speed){
       intakeMotor.set(speed);
    }


    @Override
    public void periodic(){
        getPosition();
        intakeMotorEncoder.setPosition(getPosition());
        SmartDashboard.putNumber("IntakePosition", getPosition());
    }
    
}
