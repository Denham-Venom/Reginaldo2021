// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class VisionAim extends CommandBase {
  /** Creates a new AimAndShoot. */
  private final DriveTrain drive;
  private final Shooter shoot;

  static final NetworkTable lightData = NetworkTableInstance.getDefault().getTable("limelight");
  static final NetworkTableEntry tv = lightData.getEntry("tv");
  static final NetworkTableEntry tx = lightData.getEntry("tx");
  static final NetworkTableEntry ty = lightData.getEntry("ty");
  static final NetworkTableEntry ta = lightData.getEntry("ta");
  static final NetworkTableEntry ledMode = lightData.getEntry("ledMode");

  static final ShuffleboardTab LIME = Shuffleboard.getTab("Tuning");
  static final NetworkTableEntry steerP = LIME.add("S P", 0).getEntry();
  static final NetworkTableEntry steerF = LIME.add("S F", 0).getEntry();
  static final NetworkTableEntry ALLOWABLE_ERR = LIME.add("E r r", 0).getEntry();
  static final NetworkTableEntry angleP = LIME.add("A P", 0).getEntry();
  static final NetworkTableEntry angleF = LIME.add("A F", 0)  .getEntry();

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private double m_LimelightAngleCommand = 0.0;

  private long start = 0;
  private long cur = 0;

  private long startClose = 0;
  private long curClose = 0;

  public VisionAim(DriveTrain drive, Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shoot = shoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledMode.setDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    Update_Limelight_Tracking();

    if(m_LimelightHasValidTarget)
    {
      if(Math.abs(tx.getDouble(0)) > Constants.ERR) {
        if(Math.abs(tx.getDouble(0)) < 3) {
          if(startClose == 0) {
            startClose = System.currentTimeMillis();
          } else {
            curClose = System.currentTimeMillis();
          }
        } else {
          startClose = 0;
          curClose = 0;
        }
        int sign = (int) (m_LimelightSteerCommand / Math.abs(m_LimelightSteerCommand));
        double diff = sign * (curClose-startClose);
        drive.setLeftMotors(m_LimelightDriveCommand + m_LimelightSteerCommand + diff/1000);
        drive.setRightMotors(m_LimelightDriveCommand - m_LimelightSteerCommand - diff/1000);
        start = 0;
        cur = 0;
      } else {
        if(start == 0) {
          start = System.currentTimeMillis();
        } else {
          cur = System.currentTimeMillis();
        }
      }
      if(Math.abs(ty.getDouble(0)+15 - shoot.angleEncoder.getPosition()) > Constants.ERR)
      {
        SmartDashboard.putNumber("s a v", m_LimelightAngleCommand);
        shoot.setAngleMotorsSafe(m_LimelightAngleCommand);
      }
    }
    else
    {
      shoot.setAngleMotorsSafe(0);
      drive.setLeftMotors(-m_LimelightSteerCommand);
      drive.setRightMotors(m_LimelightSteerCommand);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledMode.setDouble(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cur - start > 250) return true;
    return false;
  }
//Allows for the LimeLight to constantly send data to smartdashboard to see target
  public void Update_Limelight_Tracking()
  {
    double STEER_P = steerP.getDouble(0);
    double STEER_F = steerF.getDouble(0);
    double ANGLE_P = angleP.getDouble(0);
    double ANGLE_F = angleF.getDouble(0);

    final double AUTODRIVE = 0.0;
    final double DESIRED_TARGET_AREA = 0.0;
    final double speedLimit = 0.5;
    final double angleSpeedLimit = 0.3;
    final double steerLimit = 0.25;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if(tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.25;
      m_LimelightAngleCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    double steer_cmd = tx * STEER_P + tx / Math.abs(tx) * STEER_F;
    m_LimelightSteerCommand = steer_cmd;

    if(Math.abs(steer_cmd) > steerLimit) {
      m_LimelightSteerCommand = steerLimit * m_LimelightSteerCommand / Math.abs(m_LimelightSteerCommand);
    }

    double drive_cmd = (DESIRED_TARGET_AREA - ta) * AUTODRIVE;

    if (drive_cmd > speedLimit)
    {
      drive_cmd = speedLimit;
    }
    m_LimelightDriveCommand = drive_cmd;

    double aErr = (ty+15)-shoot.angleEncoder.getPosition();
    double angle_cmd = aErr * ANGLE_P + aErr / Math.abs(aErr) * ANGLE_F;

    if(angle_cmd > angleSpeedLimit)
    {
      angle_cmd = angleSpeedLimit;
    }
    m_LimelightAngleCommand = angle_cmd;
  }
}
