package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {
    private final Drivetrain m_Drivetrain;
    private final DoubleSupplier m_Magnitude;
    private final DoubleSupplier m_Rotation;

    public DefaultDrive(Drivetrain drive, DoubleSupplier magnitude, DoubleSupplier rotation){
        m_Drivetrain = drive;
        m_Magnitude = magnitude;
        m_Rotation = rotation;
        addRequirements(m_Drivetrain);
    }

    @Override 
    public void execute(){
        m_Drivetrain.ArcadeOpenLoop(m_Magnitude.getAsDouble(), m_Rotation.getAsDouble());
        //m_Drivetrain.setCheesyishDrive(m_Magnitude.getAsDouble(), m_Rotation.getAsDouble(), false);
    }

}