// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToShoot extends ParallelCommandGroup {
  /** Creates a new AlignToShoot. */

  public AlignToShoot(SwerveSubsystem m_SwerveSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RotationalAlign(m_SwerveSubsystem), new PivotAlign(m_ShooterSubsystem));
  }
}
