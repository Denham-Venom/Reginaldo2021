// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Move;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAndIndex;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.VisionAim;
import frc.robot.commands.VisionTurnThenAim;
import frc.robot.Constants;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeStop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {

  private double green = 15;
  private double yellow = 15;
  private double blue = 15;
  private double red = 15;

  /** Creates a new AutoScore. */
  public AutoScore(DriveTrain dt, Intake intake, Shooter shoot, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double startAngle = dt.getAngle();
    addCommands(
      new InstantCommand(intake::extendIntake),
      new VisionTurnThenAim(dt, shoot),
      new ShootAndIndex(intake, shoot),
      new TurnToAngle(dt, startAngle), //resetangle
      new Move(dt, -green),//goback
      new TurnToAngle(dt, startAngle), //resetangle
      new IntakeBall(intake, Constants.INTAKE_BALL_SPEED),//getballs
      new Move(dt, yellow),//gotoyellow
      new VisionTurnThenAim(dt, shoot),//aimandshoot
      new ShootAndIndex(intake, shoot),
      new TurnToAngle(dt, startAngle), //resetangle
      new Move(dt, -yellow), //goback
      new TurnToAngle(dt, startAngle), //resetangle
      new IntakeBall(intake, Constants.INTAKE_BALL_SPEED),//getballs
      new Move(dt, blue),//gotoblue
      new VisionTurnThenAim(dt, shoot), // aimandshoot
      new ShootAndIndex(intake, shoot),
      new TurnToAngle(dt, startAngle), //resetangle
      new Move(dt, -blue), // goback
      new TurnToAngle(dt, startAngle), //resetangle
      new IntakeBall(intake, Constants.INTAKE_BALL_SPEED), // getballs
      new TurnToAngle(dt, startAngle), //resetangle
      new Move(dt, red), // gotored
      new TurnToAngle(dt, startAngle), //resetangle
      new VisionTurnThenAim(dt, shoot),//aimandshoot
      new TurnToAngle(dt, startAngle), //resetangle
      new Move(dt, -red),//goback
      new TurnToAngle(dt, startAngle), //resetangle
      new IntakeBall(intake, Constants.INTAKE_BALL_SPEED),//getballs
      new TurnToAngle(dt, startAngle), //resetangle
      new Move(dt, red),//gotored
      new TurnToAngle(dt, startAngle), //resetangle
      new VisionTurnThenAim(dt, shoot)//aimandshoot
    );
  }
}
