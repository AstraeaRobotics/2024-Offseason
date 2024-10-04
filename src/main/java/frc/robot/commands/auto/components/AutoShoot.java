// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
 
import frc.robot.commands.pivot.SetState;
import frc.robot.commands.shooter.PostIntake;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.Constants.ShooterConstants.ShooterStates;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new PrepShoot. */
  public AutoShoot(ShooterSubsystem shooterSubsystem, ShooterStates shootState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(0.25), new PostIntake(shooterSubsystem)),
      new ParallelDeadlineGroup(new WaitCommand(1.5), new SetState(shooterSubsystem, shootState)),
      new ParallelDeadlineGroup(new WaitCommand(1), new ShootNote(shooterSubsystem))
    );
  }
}
