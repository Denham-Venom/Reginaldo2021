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

  // A - Angle Motor Up
  private final JoystickButton aButton2 = new JoystickButton(copilotDriverController, Constants.ABUTTON);

  // B - Angle Motor Down
  private final JoystickButton bButton2 = new JoystickButton(copilotDriverController, Constants.BBUTTON);

  // X - Spinup Wheel
  private final JoystickButton xButton2 = new JoystickButton(copilotDriverController, Constants.XBUTTON);

  // Y - Indexer
  private final JoystickButton yButton2 = new JoystickButton(copilotDriverController, Constants.YBUTTON);

  // LB - Shooter
  private final JoystickButton lbButton2 = new JoystickButton(copilotDriverController, Constants.LBBUTTON);

  // RB - Intake
  private final JoystickButton rbButton2 = new JoystickButton(copilotDriverController, Constants.RBBUTTON);

  // LT - 
  private final JoystickButton ltButton2 = new JoystickButton(copilotDriverController, Constants.ltBUTTON);

  // RT - 
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
    // Invert Drive <3
    aButton.whenPressed(new InstantCommand(drivetrain::invertDrive, drivetrain)); 

    // Switch Gear
    bButton.whenPressed(new InstantCommand(drivetrain::hLGearSwitch, drivetrain));

    // Toggle Intake
    xButton.whenPressed(new InstantCommand(intake::toggleIntake));

    // Aim and Shoot
    yButton.whenHeld(new AimAndShoot(drivetrain, shooter, intake));

    // Turn On/Off Limelight
    ltButton.toggleWhenPressed(new StartEndCommand(() ->  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(0), () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1)));

    // Intake Motor
    rtButton.whenHeld(new StartEndCommand(() -> intake.setIntakeMotor(Constants.INTAKE_BALL_SPEED), () -> intake.setIntakeMotor(0)));

    //sets shooter velocity with shuffleboard rpm
    rbButton.whileHeld(new StartEndCommand(() -> shooter.setShooterVelocity(), 
                                            () -> shooter.setShooterMotors(0))); 

    //----------Joystick 1 Controls----------//
    
    // Angle Motor Up
    aButton2.whenHeld(new StartEndCommand(() -> shooter.setAngleMotorsSafe(Constants.ANGLE_MOTOR_SPEED), () -> shooter.setAngleMotor(0) )); 

    // Angle Motor Down
    bButton2.whenHeld(new StartEndCommand(() -> shooter.setAngleMotorsSafe(-Constants.ANGLE_MOTOR_SPEED), () -> shooter.setAngleMotor(0) )); 

    // Spinup Wheel
    xButton2.whenHeld(new StartEndCommand(() -> intake.setSpinUpMotor(Constants.SPIN_UP_SPEED), () -> intake.setSpinUpMotor(0) )); 

    // Indexer
    yButton2.whenHeld(new StartEndCommand(() -> intake.setIndexerMotor(Constants.INDEXER_SPEED), () -> intake.setIndexerMotor(0) )); 

    // Shooter
    rbButton2.whenHeld(new StartEndCommand(() -> shooter.setShooterMotors(Constants.SHOOTER_MOTOR_SPEED), () -> shooter.setShooterMotors(0) )); 

    // Intake
    lbButton2.whenHeld(new StartEndCommand(() -> intake.setIntakeMotor(Constants.INTAKE_BALL_SPEED), () -> intake.setIntakeMotor(0) )); 
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, pilotDriverController));
  }

  public void zeroShooterAngle() {
    shooter.zeroEncoder();
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