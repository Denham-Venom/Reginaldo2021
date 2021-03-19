// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
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
        new PIDController(Robot.aimP.getDouble(0), Robot.aimI.getDouble(0), Robot.aimD.getDouble(0)),
        // This should return the measurement
        () -> shooter.angleEncoder.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> (Robot.lltv.getDouble(0) == 1) ? (Robot.llty.getDouble(0) + Constants.LL_ANG) * Constants.CAM_ANG_TO_SHOOT_ANG /*- shooter.angleEncoder.getPosition()*/ : 0, //0 is when shooter is bottomed out, tv = 1 means target found
        // This uses the output
        output -> {
          double err = Robot.llty.getDouble(0);
          int sign = (int) (err / Math.abs(err));
          output = output + sign * Robot.aimF.getDouble(0);
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
    getController().setP(Robot.aimP.getDouble(0));
    getController().setI(Robot.aimI.getDouble(0));
    getController().setD(Robot.aimD.getDouble(0));
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
}
