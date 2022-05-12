package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants;

import frc.robot.lib.LazySparkMax;
import frc.robot.lib.SparkMaxFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

public class Feeder extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private final TalonSRX MainFeed;
    private final TalonSRX SubFeed;

    private final DigitalInput MagnetSensor;

    private final LazySparkMax feederMotor;
    private final RelativeEncoder feederEncoder;
    private final SparkMaxPIDController feederPID;

    // final PWMVictorSPX antijamRoller;
    private final LazySparkMax antijamRoller;

    private boolean AutoFirstRun = false;
    private boolean armed = false;
    private boolean firing = false;
    private double feederpos;

    private final Timer m_Timer;
    private final Timer m_ShootTimer;

    public Feeder() {
        feederMotor = SparkMaxFactory.createDefaultSparkMax(Constants.FeederMotor_ID);
        feederMotor.setInverted(true);
        feederPID = feederMotor.getPIDController();
        feederEncoder = feederMotor.getEncoder(Type.kHallSensor, 42);
        feederEncoder.setMeasurementPeriod(1);
        feederEncoder.setPosition(0);
        feederEncoder.setPositionConversionFactor(42);
        feederPID.setP(Constants.Feeder_kP);
        feederPID.setFF(Constants.Feeder_kF);
        feederMotor.burnFlash();

        SubFeed = new TalonSRX(Constants.SubFeeder_ID);
        MainFeed = new TalonSRX(Constants.MainFeeder_ID);

        MagnetSensor = new DigitalInput(Constants.MagnetSensor_DIO);
        //antijamRoller = new PWMVictorSPX(1);
        antijamRoller = SparkMaxFactory.createDefaultSparkMax(Constants.AntiJam_ID);

        antijamRoller.setInverted(true);

        m_Timer = new Timer();
        m_Timer.reset();

        m_ShootTimer = new Timer();
        m_ShootTimer.reset();
    }

    public void AutoFeeder(){

        antijamRoller.set(.5);
       




        Feeder_Forward();
        
        if(feederMotor.getOutputCurrent() > 32 && AutoFirstRun == true){
            feederMotor.set(-0.05);
            if(m_Timer.get() == 0){
                m_Timer.start();
            }

            if(m_Timer.hasElapsed(0.4)){
                Feeder_Stop();
                m_Timer.stop();
                m_Timer.reset();
            }
        }
        AutoFirstRun = true;
    }

    public void Feeder_Forward(){
        feederPID.setReference(Constants.FeederSpeed, ControlType.kVelocity);
    }

    public void FeederShoot(){
        feederPID.setReference(Constants.FeederShootSpeed, ControlType.kVelocity);
    }

    public void Feeder_Reverse(){
        feederPID.setReference(Constants.ReverseFeederSpeed, ControlType.kVelocity);
    }

    public void Feeder_Stop(){
        feederMotor.set(ControlType.kDutyCycle, 0.0);
        antijamRoller.set(0);
    }

    public void PreShoot(){
        
        if(MagnetSensor.get() == true && !armed){
            feederMotor.set(ControlType.kDutyCycle, 0.1);
        }else{
            armed = true;
            if(armed && !firing){
                feederMotor.set(ControlType.kVelocity, 0.0);

                if(m_ShootTimer.get() == 0){
                    m_ShootTimer.start();
                }

                SubFeed.set(ControlMode.PercentOutput, -1.0);
                MainFeed.set(ControlMode.PercentOutput, -1.0);

                if(m_ShootTimer.hasElapsed(0.25)){
                    firing = true;
                    m_ShootTimer.stop();
                    m_ShootTimer.reset();
                }
            }
        }
        
    }

    public void Shoot(boolean ShooterReady){
        if(firing && ShooterReady){
            firing = true;
            SubFeed.set(ControlMode.PercentOutput, -1.0);
            MainFeed.set(ControlMode.PercentOutput, -1.0);
            feederMotor.set(ControlType.kVelocity, 450);
        }
    }


    public void ShooterFeedIN(){
        SubFeed.set(ControlMode.PercentOutput, -1.0);
        MainFeed.set(ControlMode.PercentOutput, -1.0);
    }

    public void ShooterFeedStop(){
        SubFeed.set(ControlMode.PercentOutput, 0);
        MainFeed.set(ControlMode.PercentOutput, 0);
        armed = false;
        firing = false;
    }
  
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("armed", armed);
        SmartDashboard.putBoolean("firing", firing);
        SmartDashboard.putNumber("Revolver RAW Position", feederpos);
        SmartDashboard.putBoolean("MagnetSensor", MagnetSensor.get());
        SmartDashboard.putNumber("Feeder Current Draw", feederMotor.getOutputCurrent());
        SmartDashboard.putNumber("Feeder Current Speed", feederEncoder.getVelocity());
        // if(feederpos > 639){ ////If the revolver has gone around more than once and feederpos is over 640
        //     feederEncoder.setPosition(0); //Set back to zero to represent a complete revolution. 
        // }
        // feederpos = feederEncoder.getPosition();
        // double curr = feederpos/128; //Maximum per revolution 640, with five possible slots. Divide by 128.
        // //if(curr % 1 == 0) curr+=1; //If the value is between two whole numbers, it is in the middle of a slot. Add one to it to "bump it up" to the proper slot so this doesn't start at zero, and also to help round. (The double truncates down when it is cast to an int later on.)
        // SmartDashboard.putNumber("Current Revolver Slot", (int)curr + 1); //Truncate down the double to the whole int below it
    }
}

