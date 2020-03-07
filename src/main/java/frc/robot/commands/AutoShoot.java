package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {

    private final Shooter m_shooter;
    private final Feeder m_Feeder;
    private final Turret m_Turret;

    
    private Timer timer = new Timer();
    private boolean finished = false;

    public AutoShoot(Shooter shoot, Feeder feed, Turret turret){
        m_shooter = shoot;
        m_Feeder = feed;
        m_Turret = turret;

        timer.reset();
        addRequirements(m_shooter, m_Feeder, m_Turret);
    }

    
    @Override 
    public void execute(){
        if(timer.get() == 0){
            timer.start();
        }

        if(!timer.hasElapsed(10)){
            m_shooter.ShooterHood_toPosition(55);
            m_shooter.Shooter_Spinup_Auto();
            m_Feeder.PreShoot();
            m_Turret.turret_LimelightControl(0);
            m_Feeder.Shoot(m_shooter.Shooter_Ready());
        }

        if(timer.hasElapsed(6)){
            timer.stop();
            finished = true;
            m_Feeder.Feeder_Stop();
            m_Feeder.ShooterFeedStop();
            m_shooter.Shooter_Stop();
            m_Turret.turret_Stop();
        }

    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return finished;
  }

}