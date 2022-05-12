package frc.robot.subsystems;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.lib.LazySparkMax;
import frc.robot.lib.SparkMaxFactory;

import frc.robot.lib.geometry.Twist2d;
import frc.robot.lib.util.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain extends SubsystemBase {

  private Timer timer = new Timer();

  private final LazySparkMax LeftDriveMaster, RightDriveMaster, LeftSlave1, LeftSlave2, RightSlave1, RightSlave2;

  private SlewRateLimiter speedLimiter = new SlewRateLimiter(2.7);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(2.7);

  private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
    sparkMax.setInverted(!left);
    sparkMax.setIdleMode(IdleMode.kCoast);
    sparkMax.setSmartCurrentLimit(40);
    sparkMax.setClosedLoopRampRate(Constants.kDriveVoltageRampRate);
  }

  public Drivetrain() {

    LeftDriveMaster = SparkMaxFactory.createDefaultSparkMax(Constants.LeftDriveMaster_ID);
    configureSpark(LeftDriveMaster, true, true);

    LeftSlave1 = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.LeftDriveSlave1_ID, LeftDriveMaster);
    configureSpark(LeftSlave1, true, false);

    LeftSlave2 = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.LeftDriveSlave2_ID, LeftDriveMaster);
    configureSpark(LeftSlave2, true, false);

    RightDriveMaster = SparkMaxFactory.createDefaultSparkMax(Constants.RightDriveMaster_ID);
    configureSpark(RightDriveMaster, false, true);

    RightSlave1 = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.RightDriveSlave1_ID, RightDriveMaster);
    configureSpark(RightSlave1, false, false);

    RightSlave2 = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.RightDriveSlave2_ID, RightDriveMaster);
    configureSpark(RightSlave2, false, false);

    timer.reset();
  }

  double drive_math(double input){
    double out;
    double sing;
   
    if(input < 0){
        sing = -1;
        input = -input;
    }else {
        sing = 1;
    }
   
    if (Math.abs(input) < .15){
        out = 0;
    }else{
        if(Math.abs(input) < .80){
            out = input * 0.55;
        }else {
            out = input;
        }
    } 
    //return out*sing;
    return Math.sin(out * sing);

  }


  public void ArcadeOpenLoop(double magnitude, double rotation){

    magnitude = speedLimiter.calculate(magnitude * 0.4);
    rotation = rotLimiter.calculate(rotation * 0.4);

    double leftPower =  magnitude - rotation;
    double rightPower = magnitude + rotation;

    RightDriveMaster.set((rightPower));
    LeftDriveMaster.set((leftPower));
  }

  public void setOpenLoop(DriveSignal signal) {
      LeftDriveMaster.set(signal.getLeft());
      RightDriveMaster.set(signal.getRight());
  }

  public void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
    throttle = speedLimiter.calculate(throttle);
    wheel = speedLimiter.calculate(wheel);

    if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
        throttle = 0.0;
        quickTurn = true;
    }

    if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
        wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
