package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {

    private final TalonFX Flywheel;
    private final TalonSRX Hood;

    private double FlywheelTarget;
    private double HoodPos;

    private boolean hold = false;

    private boolean PID_SETLOW;
    private boolean PID_SETHIGH;

    private enum FieldPosition {

    }

    public Shooter() {

        Flywheel = new TalonFX(Constants.Shooter_ID);

        Hood = new TalonSRX(Constants.ShooterHood_ID);
        Hood.setSensorPhase(false);
        Hood.setInverted(false);
        Hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants.CTRE_TimeoutMS);

        Hood.config_kP(0, Constants.Hood_kP, Constants.CTRE_TimeoutMS);

        Flywheel.configNominalOutputForward(0, Constants.CTRE_TimeoutMS);
		Flywheel.configNominalOutputReverse(0, Constants.CTRE_TimeoutMS);
		Flywheel.configPeakOutputForward(1, Constants.CTRE_TimeoutMS);
        Flywheel.configPeakOutputReverse(-1, Constants.CTRE_TimeoutMS);
        Flywheel.configClosedloopRamp(Constants.Shooter_RampRate);
    
        Flywheel.config_kF(0, Constants.Shooter_kF);
        Flywheel.config_kP(0, Constants.Shooter_kP);
        
        Flywheel.setSensorPhase(true);
        Flywheel.setInverted(true);

    }

    public double ShooterHood_GetPositionDegrees(){
        return (Hood.getSelectedSensorPosition() / (4096 / 360)) / 4.84375;
    }

    public double ShooterHood_DegreestoSensorUnits(int target){
        return ((target * 4096) / 360) * 4.84375;
    }

    public void ShooterHood_toPosition(int targetdeg){
        Hood.set(ControlMode.Position, ShooterHood_DegreestoSensorUnits(targetdeg));
    }

    public double ShooterHood_GetPositionTicks(){
        return Hood.getSelectedSensorPosition();
    }

    public boolean ShooterHood_inBounds(){
        if(this.ShooterHood_GetPositionDegrees() < 60 && this.ShooterHood_GetPositionDegrees() > 5)
            return true;
        else
            return false;
    }


    public void ShooterHood_OpenLoop(double input){
        if(ShooterHood_inBounds()){
            Hood.set(ControlMode.PercentOutput, input * 0.4);
        }else{
            if((this.ShooterHood_GetPositionDegrees() > 60 && input < 0) || (this.ShooterHood_GetPositionDegrees() < 5 && input > 0)){
                Hood.set(ControlMode.PercentOutput, input * 0.4);
            }else{
                this.ShooterHood_Stop();
            }
        }
    }

    public void ShooterHood_FWD(){
        Hood.set(ControlMode.PercentOutput, 0.2);
        hold = false;
    }

    public void ShooterHood_RVS(){
        Hood.set(ControlMode.PercentOutput, -0.2);
        hold = false;
    }

    public void ShooterHood_Stop(){
        if(!hold){
            HoodPos = Hood.getSelectedSensorPosition();
            hold = true;
        }
        if(hold){
            Hood.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public double speedConvert(double speed){
        return (speed * (2048 / 600)); 
    }

    public void Shooter_Spinup_Auto(){
        Flywheel.set(ControlMode.Velocity, this.speedConvert(Constants.ShooterSpeedAuto));
    }

    public void Shooter_Spinup(){
        if(ShooterHood_GetPositionDegrees() < 40){
            if(!PID_SETLOW){
                Flywheel.config_kF(0, Constants.Shooter_kF_LOW);
                Flywheel.config_kP(0, Constants.Shooter_kP_LOW);
                PID_SETLOW = true;
                PID_SETHIGH = false;
            }
            Flywheel.set(ControlMode.Velocity, this.speedConvert(Constants.ShooterSpeedClose));
        }else{
            if(!PID_SETHIGH){
                Flywheel.config_kF(0, Constants.Shooter_kP);
                Flywheel.config_kP(0, Constants.Shooter_kP);
                PID_SETHIGH = true;
                PID_SETLOW = false;
            }
            Flywheel.set(ControlMode.Velocity, this.speedConvert(Constants.ShooterSpeedTrench));  
        }
    }

    public void Shooter_Spinup_Slow(){
        Flywheel.set(ControlMode.Velocity, this.speedConvert(3000));
    }

    public boolean Shooter_Ready(){
        if(Flywheel.getControlMode() == ControlMode.Velocity){
            if(Math.abs((Flywheel.getClosedLoopError()) * 60 / 2048 ) < 70 ){
                return true;
            }else{
                return false;
            }
        }else{
            return false;
        }

    }

    public void Shooter_OpenLoop(){
        Flywheel.set(ControlMode.PercentOutput, 0.5);
    }

    public void Shooter_Stop(){
        Flywheel.set(ControlMode.PercentOutput, 0.0);
    }

    public double Shooter_GetRPM(){
        //600 = 100ms in on 1minute
        //2048 = pulses of encoder in 1 revolution of motor shaft        
        return Flywheel.getSelectedSensorVelocity() * 600 / 2048;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel RPM", this.Shooter_GetRPM());
        SmartDashboard.putBoolean("Shooter Ready", this.Shooter_Ready());
        SmartDashboard.putNumber("Shooter Current Draw", Flywheel.getSupplyCurrent());

        if(Flywheel.getControlMode() == ControlMode.Velocity){
            SmartDashboard.putNumber("Shooter Target Diff", Math.abs((Flywheel.getClosedLoopError()) * 60 / 2048 ));
        }

        SmartDashboard.putNumber("Hood Position Degrees", this.ShooterHood_GetPositionDegrees());

    }
}