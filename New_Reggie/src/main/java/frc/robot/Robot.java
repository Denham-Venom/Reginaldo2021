// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
  public static final NetworkTableEntry steerP = tuning.add("S P", 0).getEntry();
  public static final NetworkTableEntry steerI = tuning.add("S I", 0).getEntry();
  public static final NetworkTableEntry steerD = tuning.add("S D", 0).getEntry();
  public static final NetworkTableEntry steerF = tuning.add("S F", 0).getEntry();
  public static final NetworkTableEntry aimP = tuning.add("A P", 0).getEntry();
  public static final NetworkTableEntry aimI = tuning.add("A I", 0).getEntry();
  public static final NetworkTableEntry aimD = tuning.add("A D", 0).getEntry();
  public static final NetworkTableEntry aimF = tuning.add("A F", 0).getEntry();

  public static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  public static final NetworkTableEntry lltx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  public static final NetworkTableEntry llty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  public static final NetworkTableEntry lltv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  public static final NetworkTableEntry ledMode = limelight.getEntry("ledMode");

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("tx", lltx.getDouble(0));
    SmartDashboard.putNumber("ty", llty.getDouble(0));
    SmartDashboard.putNumber("tv", lltv.getDouble(0));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
