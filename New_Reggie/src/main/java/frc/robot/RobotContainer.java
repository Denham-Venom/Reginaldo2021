// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

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
  private final Indexer index = new Indexer();
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
    xButton.toggleWhenPressed(new InstantCommand(intake::toggleIntake));
    aButton.whenPressed(new InstantCommand(drivetrain::invertDrive, drivetrain));
    bButton.whenPressed(new InstantCommand(drivetrain::hLGearSwitch, drivetrain));
    rtButton.whileHeld(new IntakeBall(intake, Constants.INTAKE_BALL_SPEED));

    ltButton2.whileHeld(new IntakeBall(intake, -Constants.INTAKE_BALL_SPEED));
    rtButton2.whileHeld(new IntakeBall(intake, Constants.INTAKE_BALL_SPEED));
    bButton2.whileHeld(new ShooterAngle(shooter, -Constants.ANGLE_MOTOR_SPEED));
    aButton2.whileHeld(new ShooterAngle(shooter, Constants.ANGLE_MOTOR_SPEED));
    xButton2.whileHeld(new StartEndCommand(() -> index.setSpinUpMotor(-Constants.SPIN_UP_SPEED), () -> index.setSpinUpMotor(0), index));
    rbButton2.whileHeld(new Shoot(shooter));
    lbButton2.whileHeld(new Index(index));
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, pilotDriverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
  //}
}