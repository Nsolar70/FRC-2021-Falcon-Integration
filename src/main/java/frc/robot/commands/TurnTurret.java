
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.Robot;
import frc.robot.limelight.ControlMode.LedMode;

public class TurnTurret extends CommandBase {
 
  private final Turret m_turret;
  boolean m_defaultTracking = true;

  // Requirements
  public TurnTurret(Turret subsystem, Boolean Tracking ) {
    m_turret = subsystem;
    addRequirements(subsystem);
    m_defaultTracking = Tracking;
  }
  

  // Init
  @Override
  public void initialize() {
    Robot.m_limeLight.setLEDMode(LedMode.kforceOn);
  }

  // Run
  @Override
  public void execute() {
    m_turret.TrackTarget();
  }

  // End
  @Override
  public void end(boolean interrupted) {
   if (m_defaultTracking == true )
    m_turret.StopUpdating();
    m_turret.stopMotion();
    Robot.m_limeLight.setLEDMode(LedMode.kforceOff);
  }

  // Finish
  @Override
  public boolean isFinished() {
    return false;
  }
}