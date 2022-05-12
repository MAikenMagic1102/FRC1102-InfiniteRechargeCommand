package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lib.AS5600Encoder;
import frc.robot.lib.Limelight;
import frc.robot.lib.Limelight.LightMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;

import frc.robot.lib.AS5600Encoder;

import javax.naming.LimitExceededException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Turret extends SubsystemBase {

    private final TalonSRX turret;
    private final AS5600Encoder turretEncoder;
    private final Limelight limelight;
    private final PIDController turretPID;
    //private final DigitalInput turnlimit;


    public Turret(){
        turret = new TalonSRX(Constants.Turret_ID);
        turretEncoder = new AS5600Encoder(turret.getSensorCollection());
        limelight = new Limelight();
        limelight.setLedMode(LightMode.eOff);
        turretPID = new PIDController(Constants.Turret_kP, 0, Constants.Turret_kD);
        //turnlimit = new DigitalInput(Constants.TurnLimit_DIO);

    }

    public void turret_LimelightControl(double input){
        limelight.setLedMode(LightMode.eOn);
        if(limelight.isTarget()){
            double turretOutput = limelight.getTx() * Constants.Turret_kP + Constants.Turret_kD;
                turret.set(ControlMode.PercentOutput, turretOutput);
        }else{
            if(0.18 > Math.abs(input))
                turret.set(ControlMode.PercentOutput, input);
        }
    }

    public void turret_Stop(){
        turret.set(ControlMode.PercentOutput, 0.0);
        limelight.setLedMode(LightMode.eOff);
    }

    public boolean turret_inBounds(){
        if(turretEncoder.getPwmPosition() > 400 && turretEncoder.getPwmPosition() < 5000){
            return true;
        }else{
            return false;
        }
    }

    public void Rotate_OpenLoop(double rotation){
        turret.set(ControlMode.PercentOutput, rotation * 0.5);
    }

    public double GetTurretLimelightDist(){
        return limelight.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance Inches", limelight.getDistance());
        SmartDashboard.putNumber("Turret Position", turretEncoder.getPwmPosition());
        SmartDashboard.putNumber("Limelight TX", limelight.getTx());
    }

}