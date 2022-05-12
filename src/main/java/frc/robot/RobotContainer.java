/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import frc.robot.commands.AutoFeeder;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.Autonomous;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final Feeder m_Feeder = new Feeder();
  private final Intake m_Intake = new Intake();
  private final Climber m_Climber = new Climber();
  private final Shooter m_Shooter = new Shooter();
  private final Turret m_Turret = new Turret();

  private final Joystick m_Driver_1 = new Joystick(0);
  private final Joystick m_Driver_2 = new Joystick(1);
  private final Joystick m_DemoOperator = new Joystick(3);
  private final XboxController m_Operator = new XboxController(2);

  private final CommandBase m_autonomousCommand = new Autonomous(m_Drivetrain, m_Feeder, m_Climber, m_Shooter, m_Turret);

  private boolean isAutoMode = false;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_Drivetrain.setDefaultCommand(
      new DefaultDrive(
        m_Drivetrain, 
        () -> m_Driver_1.getRawAxis(1) * -1, 
        () -> m_Driver_2.getRawAxis(2) * -1));


    m_Feeder.setDefaultCommand(
      new AutoFeeder(m_Feeder));

    m_Shooter.setDefaultCommand(
      new RunCommand(() -> m_Shooter.ShooterHood_OpenLoop(m_Operator.getRawAxis(1)), m_Shooter)); 

    m_Turret.setDefaultCommand(
      new RunCommand(() -> m_Turret.Rotate_OpenLoop(m_Operator.getRawAxis(2)), m_Turret));  

     m_Climber.setDefaultCommand(
       new RunCommand(() -> m_Climber.Skywalker_Control(m_Operator.getRawAxis(0) * -1), m_Climber));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void isAuto(boolean auto){
    isAutoMode = auto;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_Driver_1, 1)
      .whenPressed(new InstantCommand(m_Intake::Intake_Forward, m_Intake))
      .whenReleased(new InstantCommand(m_Intake::Intake_Stop, m_Intake));

    new JoystickButton(m_Driver_2, 1)
      .whenPressed(new InstantCommand(m_Intake::Intake_Reverse, m_Intake))
      .whenReleased(new InstantCommand(m_Intake::Intake_Stop, m_Intake));

    new JoystickButton(m_Driver_1, 11)
      .whenPressed(new InstantCommand(m_Climber::RachetServo_Home, m_Climber));
     
    new JoystickButton(m_Driver_1, 12)
      .whenPressed(new InstantCommand(m_Climber::RachetServo_Lock, m_Climber));  

    new JoystickButton(m_Driver_2, 5)
      .whenPressed(new InstantCommand(m_Climber::Climber_Forward, m_Climber))
      .whenReleased(new InstantCommand(m_Climber::Climber_Stop, m_Climber)); 

    new JoystickButton(m_Driver_2, 4)
      .whenPressed(new InstantCommand(m_Climber::Climber_toHook, m_Climber))
      .whenReleased(new InstantCommand(m_Climber::Climber_Stop, m_Climber)); 

    new JoystickButton(m_Driver_2, 3)
      .whenPressed(new InstantCommand(m_Climber::Climber_Reverse, m_Climber))
      .whenReleased(new InstantCommand(m_Climber::Climber_Stop, m_Climber)); 

    // new JoystickButton(m_Operator, Button.kBumperLeft.value)
    //   .whileHeld(new RunCommand(() -> m_Shooter.Shooter_Spinup(m_Turret.GetTurretLimelightDist()), m_Shooter))
    //   //.whenPressed(new RunCommand(() -> m_Turret.turret_LimelightControl(m_Operator.getRawAxis(4)), m_Turret))
    //   .whenReleased(new InstantCommand(m_Shooter::Shooter_Stop, m_Shooter))
    //   .whenReleased(new InstantCommand(m_Turret::turret_Stop, m_Turret));

    new JoystickButton(m_Operator, Button.kY.value)
      .whenPressed(new InstantCommand(m_Shooter::Shooter_OpenLoop, m_Shooter))
      .whenReleased(new InstantCommand(m_Shooter::Shooter_Stop, m_Shooter));  
      
    new JoystickButton(m_Operator, Button.kX.value)
      .whenPressed(new RunCommand(() -> m_Shooter.ShooterHood_toPosition(22), m_Shooter))
      .whenReleased(new InstantCommand(m_Shooter::ShooterHood_Stop, m_Shooter));      

    new JoystickButton(m_Operator, Button.kB.value)
      .whenPressed(new RunCommand(() -> m_Shooter.ShooterHood_toPosition(55), m_Shooter))
      .whenReleased(new InstantCommand(m_Shooter::ShooterHood_Stop, m_Shooter));       

    new JoystickButton(m_Operator, Button.kRightBumper.value)
      .whileHeld(new InstantCommand(m_Feeder::PreShoot, m_Feeder))
      .whileHeld(new RunCommand(() -> m_Shooter.Shooter_Spinup(m_Turret.GetTurretLimelightDist()), m_Shooter))
      //.whileHeld(new InstantCommand(m_Feeder::FeederShoot, m_Feeder))
      .whenReleased(new InstantCommand(m_Shooter::Shooter_Stop, m_Shooter))
      .whenReleased(new InstantCommand(m_Turret::turret_Stop, m_Turret))
      .whenReleased(new InstantCommand(m_Feeder::ShooterFeedStop, m_Feeder));

    new JoystickButton(m_Operator, 1)
      .whileHeld(new RunCommand(() -> m_Feeder.Shoot(m_Shooter.Shooter_Ready()), m_Feeder))
      //.whileHeld(new InstantCommand(m_Feeder::FeederShoot, m_Feeder))
      .whenReleased(new InstantCommand(m_Feeder::ShooterFeedStop, m_Feeder));
  }


  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autonomousCommand;
  }
}
