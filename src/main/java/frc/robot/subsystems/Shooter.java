package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter extends SubsystemBase {

    //private final TalonFX Flywheel;
    private final WPI_TalonFX Flywheel;
    private final TalonSRX Hood;

    private double FlywheelTarget;
    private double HoodPos;

    private boolean shooter_started = false;
    private boolean hold = false;

    private boolean PID_SETLOW;
    private boolean PID_SETHIGH;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    
    private enum FieldPosition {

    }

    public Shooter() {
        kFF = 0.0509;
        kP = 2.5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kMaxOutput = 1.0;
        kMinOutput = -1.0;
        maxRPM = 6300;     // free speed of Falcon 500 is listed as 6380

        Flywheel = new WPI_TalonFX(Constants.Shooter_ID);
        Flywheel.configFactoryDefault();
        Flywheel.setNeutralMode(NeutralMode.Coast);

        Hood = new TalonSRX(Constants.ShooterHood_ID);
        Hood.setSensorPhase(false);
        Hood.setInverted(true);
        Hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants.CTRE_TimeoutMS);
        
        Hood.config_kP(0, Constants.Hood_kP, Constants.CTRE_TimeoutMS);

        Flywheel.configNominalOutputForward(0, Constants.CTRE_TimeoutMS);
		Flywheel.configNominalOutputReverse(0, Constants.CTRE_TimeoutMS);
		Flywheel.configPeakOutputForward(1, Constants.CTRE_TimeoutMS);
        Flywheel.configPeakOutputReverse(-1, Constants.CTRE_TimeoutMS);
    
        Flywheel.config_kF(0, kFF);
        Flywheel.config_kP(0, kP);
        
        Flywheel.setSensorPhase(true);
        Flywheel.setInverted(true);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

    }

    public double ShooterHood_GetPositionDegrees(){
        return ((Hood.getSelectedSensorPosition() / (4096 / 360)) / 4.84375) + 90;
    }

    public double ShooterHood_DegreestoSensorUnits(int target){
        return (((target - 90) * 4096) / 360) * 4.84375;
    }

    public void ShooterHood_toPosition(int targetdeg){
        Hood.set(ControlMode.Position, ShooterHood_DegreestoSensorUnits(targetdeg));
    }

    public double ShooterHood_GetPositionTicks(){
        return Hood.getSelectedSensorPosition();
    }



    public void ShooterHood_OpenLoop(double input){
            if(!(this.ShooterHood_GetPositionDegrees() > 76 && input < 0) && !(this.ShooterHood_GetPositionDegrees() < 30 && input > 0)){
                Hood.set(ControlMode.PercentOutput, input * 0.4);
            }else{
                this.ShooterHood_Stop();
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
        return (speed / 600) * 2048; 
    }

    public void Shooter_Spinup_Auto(){
        Flywheel.set(ControlMode.Velocity, this.speedConvert(Constants.ShooterSpeedAuto));
        ShooterHood_toPosition(65);
    }

    public void ShooterStartup(){
        if(!shooter_started){
            Flywheel.set(ControlMode.Velocity, this.speedConvert(1000));
            shooter_started = true;
        }
    }

    public void Shooter_Spinup(double distance){
        // if(distance > 80 && distance < 120){
        //     ShooterHood_toPosition(63);
        // }

        // if(distance > 120 && distance < 170){
        //     ShooterHood_toPosition(68);
        // }
        Flywheel.set(ControlMode.Velocity, this.speedConvert(2300));
    }

    public void Shooter_Spinup_Slow(){
        Flywheel.set(ControlMode.Velocity, this.speedConvert(3000));
    }

    public boolean Shooter_Ready(){
        if(Flywheel.getControlMode() == ControlMode.Velocity){
            if(Math.abs((Flywheel.getClosedLoopError()) * 600 / 2048 ) < 20 ){
                return true;
            }else{
                return false;
            }
        }else{
            return false;
        }

    }

    public void Shooter_OpenLoop(){
        Flywheel.set(ControlMode.PercentOutput, 0.53);
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
        SmartDashboard.putNumber("Hood Position Degrees", this.ShooterHood_GetPositionDegrees());

        if(Flywheel.getControlMode() == ControlMode.Velocity){
            SmartDashboard.putNumber("Shooter Target Diff", Math.abs((Flywheel.getClosedLoopError()) * 600 / 2048 ));
            SmartDashboard.putNumber("Shooter Target", Flywheel.getClosedLoopTarget() * 600 / 2048);
        }

        
        SmartDashboard.putNumber("Hood Sensor RAW", Hood.getSelectedSensorPosition());
        if(Hood.getControlMode() == ControlMode.Position){
            SmartDashboard.putNumber("Hood Position Target", ((Hood.getClosedLoopTarget()  / (4096 / 360)) / 4.84375) + 90);
        }

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        if((p != kP)) { Flywheel.config_kP(kPIDLoopIdx, p, kTimeoutMs); kP = p; }
        if((i != kI)) { Flywheel.config_kI(kPIDLoopIdx, i, kTimeoutMs); kI = i; }
        if((d != kD)) { Flywheel.config_kD(kPIDLoopIdx, d, kTimeoutMs); kD = d; }
        if((iz != kIz)) { Flywheel.config_IntegralZone(kPIDLoopIdx, iz, kTimeoutMs); kIz = iz; }
        if((ff != kFF)) { Flywheel.config_kF(kPIDLoopIdx, ff, kTimeoutMs);; kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            Flywheel.configPeakOutputForward(max, kTimeoutMs);
            Flywheel.configPeakOutputReverse(min, kTimeoutMs);
          kMinOutput = min; kMaxOutput = max;
        }

    }
}