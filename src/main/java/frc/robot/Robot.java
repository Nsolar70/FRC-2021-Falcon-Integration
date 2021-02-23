
package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.limelight.LimeLight;
import frc.robot.limelight.ControlMode.CamMode;
import frc.robot.limelight.ControlMode.LedMode;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static IndexVariables m_indexVariables;
  public static LimeLight m_limeLight;
  public static TurretVariables m_turretVariables;
  

  // Robot Init
  @Override
  public void robotInit() {
    //creates instances of class used
    m_robotContainer = new RobotContainer();
    m_indexVariables = new IndexVariables();
    m_limeLight = m_robotContainer.getLimeLight();
    m_limeLight.setLEDMode(LedMode.kforceOff);

    // Runs cameras at 20 FPS
    UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture();
    UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture();
    cam0.setFPS(20);
    cam1.setFPS(20);
  }

  // Robot Periodic
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // Disabled Init
  @Override
  public void disabledInit() {
  }

  // Disabled Periodic
  @Override
  public void disabledPeriodic() {
    
  }

  // Auto Init
  @Override
  public void autonomousInit() {
    m_limeLight.setCamMode(CamMode.kvision);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }     
   
  }

  // Auto Periodic
  @Override
  public void autonomousPeriodic() {
  }

  // Tele Init
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Tele Periodic
  @Override
  public void teleopPeriodic() {
  }

  // Test Init
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // Test Periodic
  @Override
  public void testPeriodic() {
  }
}
