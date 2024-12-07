// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends Command {
  /** Creates a new ReverseIntake. */
  IntakeSubsystem intakeSubsystem;
  public ReverseIntake(IntakeSubsystem intakeSub) {
    intakeSubsystem = intakeSub;
    addRequirements(intakeSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.spinIntake(-0.7);
    intakeSubsystem.spinPreIntake(-0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.spinIntake(0);
    intakeSubsystem.spinPreIntake(0);
  }
}
