package frc.robot.commands;

import frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoFeeder extends CommandBase {
    private final Feeder m_Feeder;

    public AutoFeeder(Feeder feed){
        m_Feeder = feed;
        addRequirements(m_Feeder);
    }

    @Override 
    public void execute(){
        m_Feeder.AutoFeeder();
    }

}