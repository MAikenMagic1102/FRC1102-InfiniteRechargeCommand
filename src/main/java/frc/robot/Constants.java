/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int LeftDriveMaster_ID = 4;
    public static final int LeftDriveSlave1_ID = 5;
    public static final int LeftDriveSlave2_ID = 7;

    public static final int RightDriveMaster_ID = 2;
    public static final int RightDriveSlave1_ID = 6;
    public static final int RightDriveSlave2_ID = 3;

    public static final int DriveCurrentLimit = 30;

    public static final double kDriveWheelTrackWidthInches = 27.125;
    public static final double kDriveWheelDiameterInches = 6.0;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.02712;
    public static final double kTrackScrubFactor = 1.0469745223;

    public static final int FeederMotor_ID = 9;
    public static final double Feeder_kF = 0.0006;
    public static final double Feeder_kP = 0.0002;
    public static final int FeederSpeed = 110;
    public static final int ReverseFeederSpeed = -130;
    public static final int FeederShootSpeed = 550;

    public static final int AntiJam_ID = 1;
    public static final double AntiJamSpeed = 1.0;

    public static final int Intake_ID = 10;
    public static final double FWDIntakeSpeed = 0.8;
    public static final double RVSIntakeSpeed = -0.6;
    
    public static final int Climber_ID = 8;
    public static final double FWDClimberSpeed = 1.0;
    public static final double FWDClimberSlowSpeed = 0.3;    
    public static final double RVSClimberSpeed = -1.0;
    public static final double RVSClimberSlowSpeed = -0.3;

    public static final double Climber_kP = 0.05;

    public static final int ClimberServo_PWM = 0;

    public static final int ClimberTopLimit_DIO = 1;
    public static final int ClimberBottomLimit_DIO = 2;

    public static final int Skywalker_ID = 13;

    public static final int SubFeeder_ID = 11;
    public static final int MainFeeder_ID = 12;

    public static final int MagnetSensor_DIO = 0;

    public static final int Shooter_ID = 20;
    public static final double Shooter_kF_LOW = 0.135;
    public static final double Shooter_kP_LOW = 0.0492;

    public static double Shooter_kF = 0.04;
    public static double Shooter_kP = 0.01;//.0492;
    //public static final double Shooter_RampRate = 0.7;

    public static final int ShooterSpeedClose = 3000;
    public static final int ShooterSpeedAuto = 3900;
    public static final int ShooterSpeedTrench = 5000;
    public static final int ShooterSpeedFar = 5500;

    public static final int ShooterHood_ID = 15;
    public static final double Hood_kP = 0.95;
    public static final double Hood_kD = 0.0;

    public static final double HoodAngleAuto = 0;
    public static final double HoodAngleClose = 0;
    public static final double HoodAngleTrench = 0;
    public static final double HoodAngleFar = 0;

    public static final int Turret_ID = 14;
    public static final double Turret_kP = 0.07;
    public static final double Turret_kD = 0.0057;
    public static final int TurnLimit_DIO = 3;


    public static final int CTRE_TimeoutMS = 10;

    public static final double kDriveVoltageRampRate = 0.0;
}
