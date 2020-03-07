package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase {
    private final Drivetrain m_Drivetrain;

    private Timer timer = new Timer();
    private boolean finished = false;

    public DriveStraight(Drivetrain drive){

        m_Drivetrain = drive;

        timer.reset();
        addRequirements(m_Drivetrain);
    }

    @Override 
    public void execute(){
        if(timer.get() == 0){
            timer.start();
        }

        if(!timer.hasElapsed(1.5)){
            m_Drivetrain.ArcadeOpenLoop(0.4, 0.0);
        }

        if(timer.hasElapsed(1.5)){
            m_Drivetrain.ArcadeOpenLoop(0.0, 0.0);
            finished = true;
        }

    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return finished;
  }

}