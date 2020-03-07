package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import frc.robot.lib.LazySparkMax;
import frc.robot.lib.SparkMaxFactory;
import frc.robot.lib.DigitalServo;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class Climber extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private final LazySparkMax ClimberMotor;
    private final TalonSRX Skywalker;
    private final DigitalServo RachetServo;

    private final DigitalInput TopLimit;
    private final DigitalInput BottomLimit;

    private final CANEncoder ClimbEncoder;
    private final CANPIDController ClimbPID;

    public Climber() {
        ClimberMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Climber_ID);

        ClimbEncoder = ClimberMotor.getEncoder();
        ClimbPID = ClimberMotor.getPIDController();

        ClimbPID.setP(Constants.Climber_kP);

        ClimbEncoder.setPosition(0);

        RachetServo = new DigitalServo(Constants.ClimberServo_PWM);

        Skywalker = new TalonSRX(Constants.Skywalker_ID);

        TopLimit = new DigitalInput(Constants.ClimberTopLimit_DIO);
        BottomLimit = new DigitalInput(Constants.ClimberBottomLimit_DIO);
    }

    public void Climber_toHook(){
        ClimberMotor.set(ControlType.kPosition, 177);
    }

    public void Climber_Forward(){
        if(ClimberMotor.getOutputCurrent() < 18){
            if(ClimbEncoder.getPosition() > 150){
                ClimberMotor.set(ControlType.kDutyCycle, Constants.FWDClimberSlowSpeed);
            }else{
                ClimberMotor.set(ControlType.kDutyCycle, Constants.FWDClimberSpeed);
            }
        }else{
            ClimberMotor.set(ControlType.kDutyCycle, 0.0);
        }

    }

    public void Climber_Reverse(){
        if(ClimbEncoder.getPosition() < 60){
            ClimberMotor.set(ControlType.kDutyCycle, Constants.RVSClimberSlowSpeed);
        }else{
            ClimberMotor.set(ControlType.kDutyCycle, Constants.RVSClimberSpeed);
        }
    }

    public void Skywalker_Control(double inp){
        Skywalker.set(ControlMode.PercentOutput, inp);
    }
    
    public void ClimberFWDSlow(){
         ClimberMotor.set(ControlType.kDutyCycle, 0.5);
     }

    public void ClimberRVSSlow(){
         ClimberMotor.set(ControlType.kDutyCycle, -0.5);
     }

    public void Climber_Stop(){
        ClimberMotor.set(ControlType.kDutyCycle, 0.0);
    }

    public void RachetServo_Home(){
        RachetServo.setAngle(300);
    }

    public void RachetServo_Lock(){
        RachetServo.setAngle(1);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Climber Encoder Position", ClimbEncoder.getPosition());
      SmartDashboard.putBoolean("Climber TopLimit", TopLimit.get());
      SmartDashboard.putBoolean("Climber BottomLimit", BottomLimit.get());
      SmartDashboard.putNumber("Climber Current Draw", ClimberMotor.getOutputCurrent());
      SmartDashboard.putNumber("Servo Angle", RachetServo.getAngle());
    }
  }
  