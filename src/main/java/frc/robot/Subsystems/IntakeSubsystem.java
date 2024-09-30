package frc.robot.Subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    public TalonFX intakeMotor;
    public TalonFXConfiguration intakeConfig;
    public NeutralOut neutralOut;
    public VelocityVoltage velocityRequest;
    public MotionMagicVelocityVoltage motionMagicRequest;
    public double requestedIntakeSpeed;
    
    
    public CANSparkMax pitchMotor;
    public SparkPIDController pitchMotorPID;
    public DutyCycleEncoder pitchAbsoluteEncoder;
    public RelativeEncoder pitchMotorEncoder;

    public IntakeSubsystem(){
        //1 for pitch Neo, 1 Neo for driving intake rollers, 1 bore encoder
        pitchMotor = new CANSparkMax(Constants.IntakeConstants.pitchMotorID, MotorType.kBrushless);
        pitchMotorEncoder = pitchMotor.getEncoder();
        pitchMotorPID = pitchMotor.getPIDController();
        pitchAbsoluteEncoder = new DutyCycleEncoder(Constants.IntakeConstants.pitchABSEncoder);
        
        intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);
        intakeConfig = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0).withSlot(0);
        motionMagicRequest = new MotionMagicVelocityVoltage(0);

        configure();
    }


    public void configure(){
        //Pitch Config
        pitchMotor.setInverted(Constants.IntakeConstants.pitchMotorReversed);
        pitchMotor.setIdleMode(IdleMode.kBrake);

        pitchMotorPID.setP(Constants.IntakeConstants.kP_pitch);
        pitchMotorPID.setI(Constants.IntakeConstants.kI_pitch);
        pitchMotorPID.setD(Constants.IntakeConstants.kD_pitch);


        pitchMotorEncoder.setPositionConversionFactor(360.0 * Constants.IntakeConstants.gearRatio);
        pitchAbsoluteEncoder.setPositionOffset(0.2225);


        //Intake Config
        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorReversed);

        intakeConfig.Slot0.kS = Constants.IntakeConstants.kS_intake;
        intakeConfig.Slot0.kV = Constants.IntakeConstants.kV_intake;
        intakeConfig.Slot0.kA = Constants.IntakeConstants.kA_intake;
        intakeConfig.Slot0.kP = Constants.IntakeConstants.kP_intake;
        intakeConfig.Slot0.kI = Constants.IntakeConstants.kI_intake;
        intakeConfig.Slot0.kD = Constants.IntakeConstants.kD_intake;

        intakeConfig.MotionMagic.MotionMagicAcceleration = 100;
        intakeConfig.MotionMagic.MotionMagicJerk = 3000;

        intakeMotor.getConfigurator().apply(intakeConfig);
        neutralOut = new NeutralOut();

        

        pitchMotorEncoder.setPosition(0);
    }


    //Pitch Methods
    public void setPosition(double degrees){
        pitchMotorPID.setReference(degrees, ControlType.kPosition);
    }

    public double getPosition(){
        return pitchMotorEncoder.getPosition();
    }

    public double getAbsolutePosition()
    {
        return Math.abs((pitchAbsoluteEncoder.getAbsolutePosition()-pitchAbsoluteEncoder.getPositionOffset()))*360;
    }
    
    public void gravity(){
        pitchMotorPID.setP((0.001 * Math.sin(Math.toRadians(pitchMotorEncoder.getPosition()))) + Constants.IntakeConstants.kP_pitch);//TODO Make sure the gravity constant is working
    }


    //Intake Methods
    public void setIntakeSpeed(double rps){
        // requestedIntakeSpeed = rps;
        intakeMotor.set(rps);
    }
    
    public void intakeGetUpToSpeed()
    {
        if( requestedIntakeSpeed > 0)
        {
            intakeMotor.setControl(motionMagicRequest.withVelocity(requestedIntakeSpeed));    
        }else if(requestedIntakeSpeed<0)
        {
            intakeMotor.setControl(motionMagicRequest.withVelocity(-requestedIntakeSpeed));

        }else
        {
            setIntakeNeutralOutput();
        }
    }

    public void setIntakeNeutralOutput()
    {
        intakeMotor.setControl(neutralOut);
    }


    @Override
    public void periodic(){
        intakeGetUpToSpeed();

        if(getAbsolutePosition()<130 && getAbsolutePosition()>0)
        {
            pitchMotorEncoder.setPosition(getAbsolutePosition());
        }

        SmartDashboard.putNumber("absolute position", getAbsolutePosition());
        SmartDashboard.putNumber("IntakePosition", getPosition());
    }
    
}
