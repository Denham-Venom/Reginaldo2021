// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionAimPID extends PIDCommand {

  private final Shooter s;

  /** Creates a new VisionAimPID. */
  public VisionAimPID(Shooter shooter) {
    super(
        // The controller that the command will use
        new PIDController(Constants.SHOOT_AIM_P, Constants.SHOOT_AIM_I, Constants.SHOOT_AIM_D),
        // This should return the measurement
        () -> shooter.angleEncoder.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> (Robot.lltv.getDouble(0) == 1) ? Math.atan(Math.toRadians((85.5/12)/getDistanceToTarget())) : 0, //0 is when shooter is bottomed out, tv = 1 means target found
        output -> {
          int sign = (int) (output / Math.abs(output));
          if(Robot.tuningEnable.getBoolean(false)) {
            output = output + sign * Robot.aimF.getDouble(0);
          } else {
            output = output + sign * Constants.SHOOT_AIM_F;
          }
          shooter.setAngleMotorsSafe(output);
        });
    s = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Constants.ALLOWABLE_AIM_ERR);
  }

  @Override
  public void initialize() {
    super.initialize();
    Robot.ledMode.setDouble(0);
  }

  @Override
  public void execute() {
    super.execute();
    if(Robot.tuningEnable.getBoolean(false)) { //TODO see VisionTurnPID
      getController().setP(Robot.aimP.getDouble(0));
      getController().setI(Robot.aimI.getDouble(0));
      getController().setD(Robot.aimD.getDouble(0));
    } else {
      getController().setP(Constants.SHOOT_AIM_P);
      getController().setI(Constants.SHOOT_AIM_I);
      getController().setD(Constants.SHOOT_AIM_D);
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Robot.ledMode.setDouble(1);
    s.setAngleMotorsSafe(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  private static double getDistanceToTarget() {
    final double h1 = 1.75; //feet = 21 inches
    final double h2 = 91/12; //feet = 91 in
    final double a1 = 10; //degrees
    double a2 = Robot.llty.getDouble(0);
    return (h2-h1) / Math.tan(Math.toRadians(a1+a2)); //d = (h2-h1) / tan(a1+a2);
  }
}
