package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
    private final Drivetrain m_Drivetrain;
    private final Climber m_Climber;
    private final Double m_Magnitude;
    private final Double m_Rotation;
    private final double m_time;

    private Timer timer = new Timer();
    private boolean finished = false;

    public AutoDrive(Drivetrain drive, Climber climb, Double magnitude, Double rotation, Double time){
        m_Climber = climb;
        m_Drivetrain = drive;
        m_Magnitude = magnitude;
        m_Rotation = rotation;
        m_time = time;

        timer.reset();
        addRequirements(m_Drivetrain, m_Climber);
    }

    @Override 
    public void execute(){
        if(timer.get() == 0){
            m_Climber.RachetServo_Home();
            timer.start();
        }


        if(!timer.hasElapsed(0.25)){
            m_Climber.ClimberFWDSlow();
        }

        if(timer.hasElapsed(0.25)){
            m_Drivetrain.ArcadeOpenLoop(m_Magnitude, m_Rotation);
        }

        if(timer.hasElapsed(0.5)){
            m_Climber.Climber_Stop();
            m_Drivetrain.ArcadeOpenLoop(m_Magnitude * -1, m_Rotation);
        }

        if(timer.hasElapsed(0.75)){
            m_Climber.ClimberRVSSlow();
        }

        if(timer.hasElapsed(1)){
            m_Drivetrain.ArcadeOpenLoop(0, 0);
            m_Climber.Climber_Stop();
            timer.stop();
            finished = true;
        }
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return finished;
  }

}