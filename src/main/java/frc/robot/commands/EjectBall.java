
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class EjectBall extends CommandBase {

  private final Intake m_intake;

  // Requirements
  public EjectBall(Intake subsystem) {
    m_intake = subsystem;
    addRequirements(subsystem); 
  }

  // Init
  @Override
  public void initialize() {
  }

  // Run
  @Override
  public void execute() {
    m_intake.ejectBall();
  }

  // End
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotion();
  }

  // Finish
  @Override
  public boolean isFinished() {
    return false;
  }
}
