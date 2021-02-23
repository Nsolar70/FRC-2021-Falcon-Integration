
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  
  // Motor
  private final Spark intakeMotor = new Spark(Constants.ActuatorConstants.kIntakeMotor);
  //private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.ActuatorConstants.kTestInTakeMotor);
  
  // SmartDashboard
   final String IntakeSpeed ="IntakeSpeed";
   final String OutTakeSpeed = "OutTakeSpeed";
   final double SpeedOut = -0.5;
   final double SpeedIn = 0.5;
   private Double setSpeed;

  // Intake
  public Intake() {
  }

  // Periodic
  @Override
  public void periodic() {
  }

  // Run
  public void intakeBall() {
    double backup = SpeedIn;
    setSpeed = getPreferencesDouble(IntakeSpeed ,backup);
    intakeMotor.set(setSpeed);
  }

  public void ejectBall() {
   double backup = SpeedOut;
   setSpeed = getPreferencesDouble(OutTakeSpeed, backup);
    intakeMotor.set(setSpeed);
  }

  // Stop Motion
  public void stopMotion() {
    intakeMotor.set(0.0);
  }

  // Preferences
  private static double getPreferencesDouble(String key, double backup) {
    Preferences preferences = Preferences.getInstance();
    if(!preferences.containsKey(key)) {
      preferences.putDouble(key, backup);
    }
    return preferences.getDouble(key, backup);
  }
}