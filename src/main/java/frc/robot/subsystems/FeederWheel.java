// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederWheel extends SubsystemBase {
  // Motor
  private final WPI_VictorSPX Feeder = new WPI_VictorSPX(Constants.ActuatorConstants.kFeeder);

  // SmartDashboard
  final String IntakeSpeed ="FeederWheelSpeed";
  final double SpeedIn = -0.51;
  private  double setSpeed;

  /** Creates a new FeederWheel. */
  public FeederWheel() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Feeder
  public void RunFeederWheel() {
    double backup = SpeedIn;
    setSpeed = getPreferencesDouble(IntakeSpeed ,backup);
    Feeder.set(ControlMode.PercentOutput, setSpeed);
}

// Stop Motion
public void stopMotion() {
  Feeder.set(ControlMode.PercentOutput, 0.0);

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
