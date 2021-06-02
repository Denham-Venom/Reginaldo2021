// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoScore;
import frc.robot.commands.auto.MoveShoot;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick pilotDriverController = new Joystick(0);
  private final Joystick copilotDriverController = new Joystick(1);

  private final JoystickButton aButton = new JoystickButton(pilotDriverController, Constants.ABUTTON);
  private final JoystickButton bButton = new JoystickButton(pilotDriverController, Constants.BBUTTON);
  private final JoystickButton xButton = new JoystickButton(pilotDriverController, Constants.XBUTTON);
  private final JoystickButton yButton = new JoystickButton(pilotDriverController, Constants.YBUTTON);
  private final JoystickButton rbButton = new JoystickButton(pilotDriverController, Constants.RBBUTTON);
  private final JoystickButton lbButton = new JoystickButton(pilotDriverController, Constants.LBBUTTON);
  private final JoystickButton rtButton = new JoystickButton(pilotDriverController, Constants.rtBUTTON);
  private final JoystickButton ltButton = new JoystickButton(pilotDriverController, Constants.ltBUTTON);
 
  private final JoystickButton aButton2 = new JoystickButton(copilotDriverController, Constants.ABUTTON);
  private final JoystickButton bButton2 = new JoystickButton(copilotDriverController, Constants.BBUTTON);
  private final JoystickButton xButton2 = new JoystickButton(copilotDriverController, Constants.XBUTTON);
  private final JoystickButton yButton2 = new JoystickButton(copilotDriverController, Constants.YBUTTON);
  private final JoystickButton rtButton2 = new JoystickButton(copilotDriverController, Constants.rtBUTTON);
  private final JoystickButton ltButton2 = new JoystickButton(copilotDriverController, Constants.ltBUTTON);
  private final JoystickButton rbButton2 = new JoystickButton(copilotDriverController, Constants.RBBUTTON);
  private final JoystickButton lbButton2 = new JoystickButton(copilotDriverController, Constants.LBBUTTON);

  // The robot's subsystems and commands are defined here...
  private final DriveTrain drivetrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    xButton.whenPressed(new InstantCommand(intake::toggleIntake));
//---------------------------------------------------------------------
    //yButton.whenHeld(new VisionAim(drivetrain, shooter));
    yButton.whenHeld(new StartEndCommand(
      () -> {
        intake.setIndexerMotor(Constants.INDEXER_SPEED);
        intake.setSpinUpMotor(Constants.SPIN_UP_SPEED);
      },
      () -> {
        intake.setIndexerMotor(0);
        intake.setSpinUpMotor(0);
      }));
    
//---------------------------------------------------------------------
    aButton.whenPressed(new InstantCommand(drivetrain::invertDrive, drivetrain));
    bButton.whenPressed(new InstantCommand(drivetrain::hLGearSwitch, drivetrain));
    
    rtButton.whenHeld(new IntakeBall(intake, Constants.INTAKE_BALL_SPEED));
    rbButton.whileHeld(new StartEndCommand(() -> intake.setSpinUpMotor(Constants.SPIN_UP_SPEED), () -> intake.setSpinUpMotor(0)));
    lbButton.whileHeld(new StartEndCommand(() -> intake.setSpinUpMotor(-Constants.SPIN_UP_SPEED), () -> intake.setSpinUpMotor(0)));
    ltButton.toggleWhenPressed(new StartEndCommand(() ->  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(0), () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1)));
    
    ltButton2.whenHeld(new StartEndCommand(() -> shooter.setShooterVelocity(), () -> shooter.setShooterMotors(0)));
    rbButton2.whenHeld(new Move(drivetrain, 10));
    //rbButton2.whenHeld(new AutoScore(drivetrain, intake, shoot))
    rtButton2.whenHeld(new IntakeBall(intake, Constants.INTAKE_BALL_SPEED));
    bButton2.whileHeld(new ShooterAngle(shooter, -Constants.ANGLE_MOTOR_SPEED));
    aButton2.whileHeld(new ShooterAngle(shooter, Constants.ANGLE_MOTOR_SPEED));
    yButton2.whenHeld(new StartEndCommand(() -> drivetrain.setWithPostion(9.5), () -> drivetrain.stopMotors(), drivetrain));
    xButton2.whileHeld(new ShootAndIndex(intake, shooter));
    lbButton2.whileHeld(new StartEndCommand(() -> intake.setIndexerMotor(Constants.INDEXER_SPEED), () -> intake.setIndexerMotor(0)));
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, pilotDriverController));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoScore(drivetrain, intake, shooter);
    
  }
}
