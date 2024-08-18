package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class ShooterSubsystem extends SubsystemBase{
    
    public final TalonFX topMotor;
    public final TalonFX bottomMotor;
    public VelocityVoltage velocityRequest;
    public MotionMagicVelocityVoltage motionMagicRequest;
    public TalonFXConfiguration topConfig, bottomConfig;
    public NeutralOut neutralOut;


    public final CANSparkMax breachMotor;
    public final CANSparkMax pitchMotor;

    public final RelativeEncoder breachMotorEncoder;
    public final AbsoluteEncoder pitchMotorEncoder;
    public final SparkPIDController breachMotorPID;
    public final SparkPIDController pitchMotorPID;

    public DigitalInput breachSwitch;

    //Unit in Rotations per second
    double desiredTopVelocity = 0;
    double desiredBottomVelocity = 0;
    double shooterUpToSpeedTolerance = 5;

    public ShooterSubsystem(){
        //2 Krakens, 1 Neo pitch motor, 1 breach motor, 1 bore encoder
    topMotor = new TalonFX(Constants.ShooterConstants.topMotorID, "rio");
    bottomMotor = new TalonFX(Constants.ShooterConstants.bottomMotorID, "rio");

    breachMotor = new CANSparkMax(Constants.ShooterConstants.breachMotorID, MotorType.kBrushless);
    breachMotorEncoder = breachMotor.getEncoder();
    breachMotorPID = breachMotor.getPIDController();

    pitchMotor = new CANSparkMax(Constants.ShooterConstants.PitchEncoderID, MotorType.kBrushless);
    pitchMotorEncoder = pitchMotor.getAbsoluteEncoder();
    pitchMotorPID = pitchMotor.getPIDController();

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    motionMagicRequest = new MotionMagicVelocityVoltage(0);
        
    breachSwitch = new DigitalInput(Constants.ShooterConstants.breachSwitchPort);
    configure();
    }


    public void configure(){
    //Flywheel config
    topConfig.Slot0.kS = Constants.ShooterConstants.kS_TopShooter;
    topConfig.Slot0.kV = Constants.ShooterConstants.kV_TopShooter;
    topConfig.Slot0.kA = Constants.ShooterConstants.kA_TopShooter;
    topConfig.Slot0.kP = Constants.ShooterConstants.kP_TopShooter;
    topConfig.Slot0.kI = Constants.ShooterConstants.kI_TopShooter;
    topConfig.Slot0.kD = Constants.ShooterConstants.kD_TopShooter;

    topConfig.MotionMagic.MotionMagicAcceleration = 400;
    topConfig.MotionMagic.MotionMagicJerk = 4000;

    bottomConfig.Slot0.kS = Constants.ShooterConstants.kS_BottomShooter;
    bottomConfig.Slot0.kV = Constants.ShooterConstants.kV_BottomShooter;
    bottomConfig.Slot0.kA = Constants.ShooterConstants.kA_BottomShooter;
    bottomConfig.Slot0.kP = Constants.ShooterConstants.kP_BottomShooter;
    bottomConfig.Slot0.kI = Constants.ShooterConstants.kI_BottomShooter;
    bottomConfig.Slot0.kD = Constants.ShooterConstants.kD_BottomShooter;

    bottomConfig.MotionMagic.MotionMagicAcceleration = 400;
    bottomConfig.MotionMagic.MotionMagicJerk = 4000;

    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);

    topMotor.setInverted(Constants.ShooterConstants.topMotorReversed);
    bottomMotor.setInverted(Constants.ShooterConstants.bottomMotorReversed);
    
    neutralOut = new NeutralOut();

    //Breach Motor Config
    breachMotor.restoreFactoryDefaults();
    breachMotorPID.setP(Constants.ShooterConstants.kP_breach);
    breachMotorPID.setI(Constants.ShooterConstants.kI_breach);
    breachMotorPID.setD(Constants.ShooterConstants.kD_breach);
    breachMotor.setInverted(Constants.ShooterConstants.breachReversed);
    breachMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);
    breachMotor.setIdleMode(IdleMode.kCoast);

    //Pitch Motor Config
    pitchMotor.restoreFactoryDefaults();
    pitchMotorPID.setP(Constants.ShooterConstants.kP_pitch);
    pitchMotorPID.setI(Constants.ShooterConstants.kI_pitch);
    pitchMotorPID.setD(Constants.ShooterConstants.kD_pitch);
    pitchMotor.setInverted(Constants.ShooterConstants.pitchReversed);
    pitchMotor.setOpenLoopRampRate(5);
    pitchMotorPID.setSmartMotionMaxVelocity(5, 0);
    pitchMotor.setIdleMode(IdleMode.kBrake);


    }


    public void getUpToSpeed() {
        if (desiredTopVelocity <= 0 && desiredBottomVelocity <= 0) {
          setShootingNeutralOutput();
        } else {
          topMotor.setControl(motionMagicRequest.withVelocity(desiredTopVelocity));
          bottomMotor.setControl(motionMagicRequest.withVelocity(desiredBottomVelocity));
        }
      }
    
    public void setShootingNeutralOutput() {
        topMotor.setControl(neutralOut);
        bottomMotor.setControl(new NeutralOut());
    }
    
    public double getTopShooterVelocity() {
        return topMotor.getVelocity().getValueAsDouble();
    }
    public double getBottomShooterVelocity() {
        return bottomMotor.getVelocity().getValueAsDouble();
    }
    public boolean isTopShooterUpToSpeed() {
        return (Math.abs(getTopShooterVelocity() - desiredTopVelocity)) <= shooterUpToSpeedTolerance;
    }
    public boolean isBotomShooterUpToSpeed() {
        return (Math.abs(getBottomShooterVelocity() - desiredBottomVelocity)) <= shooterUpToSpeedTolerance;
    }
    public boolean areBothShootersUpToSpeed() {
        return isTopShooterUpToSpeed()
            && isBotomShooterUpToSpeed();
    }
    
    public void setDesiredVelocities(double desiredTopVelocity, double desiredBottomVelocity) {
        this.desiredTopVelocity = desiredTopVelocity;
        this.desiredBottomVelocity = desiredBottomVelocity;
    }
    
    //Breach Methods
    public void feed(){
        breachMotor.set(0.5);
    }




    //Pitch Methods
    public void setPosition(double degrees){
        pitchMotorPID.setReference(degrees, ControlType.kPosition);
    }

    public void home(){
        pitchMotorPID.setReference(0, ControlType.kPosition);
    }

    public void deploy(){
        setPosition(65);//TODO Check Value
    }

    public void gravity(){
        pitchMotorPID.setP((0.001 * Math.sin(Math.toRadians(pitchMotorEncoder.getPosition()))) + Constants.IntakeConstants.kP_pitch);//TODO Make sure the gravity constant is working
    }

    public double getPosition(){
        return pitchMotorEncoder.getPosition();
    }


    @Override
    public void periodic(){
        getUpToSpeed();
        getPosition();


        SmartDashboard.putNumber("ShooterPosition", getPosition());
        // SmartDashboard.putNumber("Shooter/Top/Velocity RPS", getTopShooterVelocity());
        // SmartDashboard.putNumber("Shooter/Top/Desired Velocity RPS", desiredTopVelocity);
        // SmartDashboard.putBoolean("Shooter/Top/Up to Speed", isTopShooterUpToSpeed());

        // SmartDashboard.putNumber("Shooter/Bottom/Velocity RPS", getBottomShooterVelocity());
        // SmartDashboard.putNumber("Shooter/Bottom/Desired Velocity RPS", desiredBottomVelocity);
        // SmartDashboard.putBoolean("Shooter/Bottom/Up to Speed", isBotomShooterUpToSpeed());



    }
}
