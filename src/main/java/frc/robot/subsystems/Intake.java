package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import frc.robot.lib.LazySparkMax;
import frc.robot.lib.SparkMaxFactory;

public class Intake extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private final LazySparkMax IntakeMotor;

    public Intake() {
        IntakeMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Intake_ID);
    }

    public void Intake_Forward(){
        IntakeMotor.set(Constants.FWDIntakeSpeed);
    }

    public void Intake_Reverse(){
        IntakeMotor.set(Constants.RVSIntakeSpeed);
    }

    public void Intake_Stop(){
        IntakeMotor.set(0.0);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }
  