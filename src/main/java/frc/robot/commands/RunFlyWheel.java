
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel;


public class RunFlyWheel extends CommandBase {
  
  private final FlyWheel m_flyWheel;
  boolean m_stopMotor = false;
  
  // Requirements
  public RunFlyWheel(FlyWheel subsystem, Boolean stopMotor) {
    m_flyWheel = subsystem;
      addRequirements(subsystem);
      m_stopMotor = stopMotor;
  }

  // Init
  @Override
  public void initialize() {
  }

  // Run
  @Override
  public void execute() {
    m_flyWheel.RunFlyWheel();
  }

  // End
  @Override
  public void end(boolean interrupted) {
    if (m_stopMotor == true) {
      m_flyWheel.stopMotion();
    }
    


  }

  // Finish
  @Override
  public boolean isFinished() {
    return false;
  }
}