package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AbortIntake extends Command {
  /** Creates a new AbortIntake. */

  IntakeSubsystem m_intake;

  public AbortIntake(IntakeSubsystem m_intake) {
    this.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.spinIntake(0);
    m_intake.spinPreIntake(0);
  }
}
