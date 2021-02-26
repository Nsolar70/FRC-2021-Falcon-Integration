/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeederWheel;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoShoot.
   */
  public AutoShoot(DriveTrain m_driveTrain, FlyWheel m_flyWheel, Indexer m_indexer, Turret m_turret, FeederWheel m_feederWheel) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoDriveForward(m_driveTrain).withTimeout(0.25),
      new HomeTurret(m_turret).withTimeout(1.2), new TurnTurret(m_turret, true).withTimeout(2),
      new ManualyShoot(m_flyWheel, m_indexer, m_feederWheel).withTimeout(5), new AutoDriveBackward(m_driveTrain).withTimeout(0.6));
  }
}
