// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoScore;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Joystick 0
  private final Joystick pilotDriverController = new Joystick(0);

  // A - Invert Drive Direction
  private final JoystickButton aButton = new JoystickButton(pilotDriverController, Constants.ABUTTON);

  // B - Switch Gear (H/L)
  private final JoystickButton bButton = new JoystickButton(pilotDriverController, Constants.BBUTTON);

  // X - Toggle Intake Extension
  private final JoystickButton xButton = new JoystickButton(pilotDriverController, Constants.XBUTTON);

  // Y - Aim and Shoot (Vision)
  private final JoystickButton yButton = new JoystickButton(pilotDriverController, Constants.YBUTTON);

  // LB - Test Control
  private final JoystickButton lbButton = new JoystickButton(pilotDriverController, Constants.LBBUTTON);

  // RB - Test Control
  private final JoystickButton rbButton = new JoystickButton(pilotDriverController, Constants.RBBUTTON);
  
  // LT - Toggle Limelight Mode
  private final JoystickButton ltButton = new JoystickButton(pilotDriverController, Constants.ltBUTTON);

  // RT - Intake
  private final JoystickButton rtButton = new JoystickButton(pilotDriverController, Constants.rtBUTTON);
  
 
  // Joystick 1
  private final Joystick copilotDriverController = new Joystick(1);

  // A - Spinup Out
  private final JoystickButton aButton2 = new JoystickButton(copilotDriverController, Constants.ABUTTON);

  // B - Spinup In
  private final JoystickButton bButton2 = new JoystickButton(copilotDriverController, Constants.BBUTTON);

  // X - Shoot and Index
  private final JoystickButton xButton2 = new JoystickButton(copilotDriverController, Constants.XBUTTON);

  // Y - Aim and Shoot
  private final JoystickButton yButton2 = new JoystickButton(copilotDriverController, Constants.YBUTTON);

  // LB - Index Out
  private final JoystickButton lbButton2 = new JoystickButton(copilotDriverController, Constants.LBBUTTON);

  // RB - Index In
  private final JoystickButton rbButton2 = new JoystickButton(copilotDriverController, Constants.RBBUTTON);

  // LT - Intake Out
  private final JoystickButton ltButton2 = new JoystickButton(copilotDriverController, Constants.ltBUTTON);

  // RT - Intake In
  private final JoystickButton rtButton2 = new JoystickButton(copilotDriverController, Constants.rtBUTTON);


  // Subsystems
  private final DriveTrain drivetrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands
    setDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //----------Joystick 0 Controls----------//
    aButton.whenPressed(new InstantCommand(drivetrain::invertDrive, drivetrain));
    bButton.whenPressed(new InstantCommand(drivetrain::hLGearSwitch, drivetrain));
    xButton.whenPressed(new InstantCommand(intake::toggleIntake));
    yButton.whenHeld(new AimAndShoot(drivetrain, shooter, intake));
    ltButton.toggleWhenPressed(new StartEndCommand(() ->  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(0), () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1)));
    rtButton.whenHeld(new StartEndCommand(() -> intake.setIntakeMotor(Constants.INTAKE_BALL_SPEED), () -> intake.setIntakeMotor(0)));


    //----------Joystick 1 Controls----------//
    aButton2.whileHeld(new StartEndCommand(() -> intake.setSpinUpMotor(-Constants.SPIN_UP_SPEED), () -> intake.setSpinUpMotor(0)));
    bButton2.whileHeld(new StartEndCommand(() -> intake.setSpinUpMotor(Constants.SPIN_UP_SPEED), () -> intake.setSpinUpMotor(0)));
    xButton2.whenHeld(new ShootAndIndex(intake, shooter));
    yButton2.whenHeld(new AimAndShoot(drivetrain, shooter, intake));
    lbButton2.whileHeld(new StartEndCommand(() -> intake.setIndexerMotor(-Constants.INDEXER_SPEED), () -> intake.setIndexerMotor(0)));
    rbButton2.whileHeld(new StartEndCommand(() -> intake.setIndexerMotor(Constants.INDEXER_SPEED), () -> intake.setIndexerMotor(0)));
    ltButton2.whileHeld(new StartEndCommand(() -> intake.setIntakeMotor(-Constants.INTAKE_BALL_SPEED), () -> intake.setIntakeMotor(0)));
    rtButton2.whileHeld(new StartEndCommand(() -> intake.setIntakeMotor(Constants.INTAKE_BALL_SPEED), () -> intake.setIntakeMotor(0)));


    //----------Test Controls----------//
    lbButton.whenHeld(new VisionTurnThenAim(drivetrain, shooter)); //vision
    rbButton.whileHeld(new StartEndCommand(() -> shooter.setShooterVelocity(), 
                                           () -> shooter.setShooterMotors(0))); //sets shooter velocity with shuffleboard rpm
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
