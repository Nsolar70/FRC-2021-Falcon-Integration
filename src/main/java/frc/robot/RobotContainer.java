
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.EjectBall;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ManualyIndexDown;
import frc.robot.commands.ManualyIndexUp;
import frc.robot.commands.ManualyShoot;
import frc.robot.commands.TurnTurret;
import frc.robot.limelight.LimeLight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeederWheel;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;



public class RobotContainer {

  // Subsystems
   private final DriveTrain m_driveTrain = new DriveTrain();
   private final Intake m_intake = new Intake();
   private final Turret m_turret = new Turret();
   private final Indexer m_indexer = new Indexer();
   private final FlyWheel m_flyWheel = new FlyWheel();
   private final Climber m_climber = new Climber();
   private final FeederWheel m_feederWheel = new FeederWheel();

  // The Xbox Controller
   XboxController m_driverController = new XboxController(OIConstants.kDriveJoyStick);
   XboxController m_actuatorController = new XboxController(OIConstants.kActuatorJoyStick);
  
 // Commands
   private final CommandBase m_intakeCommand = new IntakeBall(m_intake);
  // private final CommandBase m_flywheelCommand = new RunFlyWheel(m_flyWheel);
  private final CommandBase m_turretCommand = new TurnTurret(m_turret, true);
  private final CommandBase m_ejectBallCommand = new EjectBall(m_intake);
  private final CommandBase m_manualyIndexUp = new ManualyIndexUp(m_indexer);
  // private final CommandBase m_indexDownCommand = new ManualyIndexDown(m_indexer);
  private final CommandBase m_manualyShoot = new ManualyShoot(m_flyWheel, m_indexer, m_feederWheel);
  private final CommandBase m_manualyIndexDown = new ManualyIndexDown(m_indexer);
  private final CommandBase m_autoShoot = new AutoShoot(m_driveTrain, m_flyWheel, m_indexer, m_turret, m_feederWheel);

  // LimeLight
   private final LimeLight limeLight = new LimeLight();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // A split-stick drive train command, with forward/backward controlled by the
    // left
    // hand, and turning controlled by the right.
    m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.drive(m_driverController.getY(GenericHID.Hand.kRight),
            m_driverController.getX(GenericHID.Hand.kRight), m_driverController.getX(GenericHID.Hand.kLeft)),
        m_driveTrain));

        //default command for the indexer to run with the intake
        m_indexer.setDefaultCommand(new RunCommand(
          () -> m_indexer.SetDefaultCall(), m_indexer));

          m_climber.setDefaultCommand(new RunCommand(
            () -> m_climber.DefaultRun(m_driverController.getTriggerAxis(GenericHID.Hand.kRight),m_driverController.getTriggerAxis(GenericHID.Hand.kLeft)),
            m_climber));
    // Configure the button bindings
    configureButtonBindings();
      

  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    // Run instant command intake when the 'Back' button is pressed
    new JoystickButton(m_actuatorController, Button.kBack.value).whileHeld(m_intakeCommand);
    // Run Flywheel command when the 'B' button is pressed
   // new JoystickButton(m_actuatorController, Button.kB.value).toggleWhenPressed(m_flywheelCommand);
    // Run Eject Ball command when the 'start' button is pressed
    new JoystickButton(m_actuatorController, Button.kStart.value).whileHeld(m_ejectBallCommand);
    // Run Manual shooting when the y button is pressed
    new JoystickButton(m_actuatorController,Button.kY.value).toggleWhenPressed(m_manualyShoot);
    // Enable Vision Tracking
    new JoystickButton(m_actuatorController,Button.kB.value).toggleWhenPressed(m_turretCommand);
    // Run Indexer Manual Up/Down when the Right/Left Bummers are pressed
   new JoystickButton(m_actuatorController, Button.kBumperRight.value).whileHeld(m_manualyIndexUp);
   new JoystickButton(m_actuatorController, Button.kBumperLeft.value).whileHeld(m_manualyIndexDown);

  }
//
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return m_autoShoot;
  }

  public LimeLight getLimeLight() {
    return limeLight;
  }
}
