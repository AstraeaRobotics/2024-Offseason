// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants.ShooterStates;
import frc.robot.commands.pivot.SetState;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndPivot extends SequentialCommandGroup {
  /** Creates a new AlignAndPivot. */
  public AlignAndPivot(SwerveSubsystem m_SwerveSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelDeadlineGroup(new WaitCommand(1), new RotationalAlign(m_SwerveSubsystem)), 
    new ParallelDeadlineGroup(new WaitCommand(0.75), new PivotAlign(m_ShooterSubsystem))
    // new ParallelDeadlineGroup(new WaitCommand(1.5), new ShootNote(m_ShooterSubsystem))
    );
  }
}
