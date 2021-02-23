/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.limelight.ControlMode.Snapshot;
import frc.robot.subsystems.Indexer;

public class autoShootIndexer extends CommandBase {
  /**
   * Creates a new autoShootIndexer.
   */
  private final Indexer m_indexer;

  public autoShootIndexer(Indexer subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_limeLight.setSnapshot(Snapshot.kon);
  }
     
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_indexVariables.isIntakeActive = false;
    m_indexer.AutoShoot();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.StopMotion();
    Robot.m_limeLight.setSnapshot(Snapshot.koff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}