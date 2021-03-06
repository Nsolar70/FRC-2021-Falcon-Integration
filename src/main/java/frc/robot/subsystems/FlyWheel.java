
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EncoderConstants;


public class FlyWheel extends SubsystemBase {

  //Pulling the Estimated Distance off the sufffleboard
  
  // Motor
   private final WPI_TalonFX FlyWheel = new WPI_TalonFX(Constants.ActuatorConstants.kFlyWheel);
   

  // SmartDashboard
   final String IntakeSpeed ="FlyWheelSpeed";
   final double SpeedIn = -1.0;
   private  double setSpeed;
   final double EstimateDistance = SmartDashboard.getNumber("Estimate Distance", 0);
   private double targetVelocity_UnitsPer100ms;
   
  

  // Fly Wheel
   public FlyWheel() {

    

    /* Velocity Closed Loop */

			/**
			 * Convert 500 RPM to units / 100ms.
			 * 2048 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
    
    	/* 500 RPM in either direction */
      //FlyWheel.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
      
      /* Config the Velocity closed loop gains in slot0 */
    /*
		FlyWheel.config_kF(EncoderConstants.kPIDLoopIdx, Constants.kGains_Velocit.kF, EncoderConstants.kTimeoutMs);
		FlyWheel.config_kP(EncoderConstants.kPIDLoopIdx, Constants.kGains_Velocit.kP, EncoderConstants.kTimeoutMs);
		FlyWheel.config_kI(EncoderConstants.kPIDLoopIdx, Constants.kGains_Velocit.kI, EncoderConstants.kTimeoutMs);
    FlyWheel.config_kD(EncoderConstants.kPIDLoopIdx, Constants.kGains_Velocit.kD, EncoderConstants.kTimeoutMs);
    */
  }

  // Periodic
  @Override
  public void periodic() {

   
  }
  
  
  // Flywheel
  public void RunFlyWheel() {
    double backup = SpeedIn;
    setSpeed = getPreferencesDouble(IntakeSpeed ,backup);
    FlyWheel.set(ControlMode.PercentOutput, setSpeed);
/*
  if (EstimateDistance > 0 && EstimateDistance < 7.5 ) {
    targetVelocity_UnitsPer100ms =  2000.0 * 2048.0 / 600.0;
   }

   if (EstimateDistance > 7.5 && EstimateDistance < 12.5 ) {
    targetVelocity_UnitsPer100ms =  2000.0 * 2048.0 / 600.0;
   }

   if (EstimateDistance > 12.5 && EstimateDistance < 17.5) {
    targetVelocity_UnitsPer100ms =  2000.0 * 2048.0 / 600.0;
   }

   if (EstimateDistance > 17.5 && EstimateDistance < 22.5 ) {
    targetVelocity_UnitsPer100ms =  2000.0 * 2048.0 / 600.0;
   }  */

  }

  // Stop Motion
  public void stopMotion() {
    FlyWheel.set(ControlMode.PercentOutput,0.0);
    

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