package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

import frc.robot.lib.LazySparkMax;
import frc.robot.lib.SparkMaxFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class Feeder extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private final TalonSRX MainFeed;
    private final TalonSRX SubFeed;

    private final DigitalInput MagnetSensor;

    private final LazySparkMax feederMotor;
    private final CANEncoder feederEncoder;
    private final CANPIDController feederPID;

    private boolean AutoFirstRun = false;
    private boolean armed = false;
    private boolean firing = false;

    private final Timer m_Timer;
    private final Timer m_ShootTimer;

    public Feeder() {
        feederMotor = SparkMaxFactory.createDefaultSparkMax(Constants.FeederMotor_ID);
        feederMotor.setInverted(true);
        feederPID = feederMotor.getPIDController();
        feederEncoder = feederMotor.getEncoder();

        feederEncoder.setPosition(0);

        feederPID.setP(Constants.Feeder_kP);
        feederPID.setFF(Constants.Feeder_kF);
        feederMotor.burnFlash();

        SubFeed = new TalonSRX(Constants.SubFeeder_ID);
        MainFeed = new TalonSRX(Constants.MainFeeder_ID);

        MagnetSensor = new DigitalInput(Constants.MagnetSensor_DIO);

        m_Timer = new Timer();
        m_Timer.reset();

        m_ShootTimer = new Timer();
        m_ShootTimer.reset();
    }

    public void AutoFeeder(){

        Feeder_Forward();
        
        if(feederMotor.getOutputCurrent() > 38 && AutoFirstRun == true){
            feederMotor.set(ControlType.kDutyCycle, -0.05);
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
        feederMotor.set(ControlType.kVelocity, Constants.FeederSpeed);
    }

    public void FeederShoot(){
        feederMotor.set(ControlType.kVelocity, Constants.FeederShootSpeed);
    }

    public void Feeder_Reverse(){
        feederMotor.set(ControlType.kVelocity, Constants.ReverseFeederSpeed);
    }

    public void Feeder_Stop(){
        feederMotor.set(ControlType.kDutyCycle, 0.0);
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
            SubFeed.set(ControlMode.PercentOutput, -1.0);
            MainFeed.set(ControlMode.PercentOutput, -1.0);
            feederMotor.set(ControlType.kVelocity, 550);
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

        SmartDashboard.putBoolean("MagnetSensor", MagnetSensor.get());
      // This method will be called once per scheduler run
    }
  }
  