
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Turret extends SubsystemBase {
  
  // Motor
  private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.ActuatorConstants.kTurretMotor);
  //Limit Switch
  private final DigitalInput homeSwitch = new DigitalInput(Constants.SensorContants.kTurretHome);
 
  // SmartDashboard
  final String TurretSpeed ="TurretSpeed";
  final double SpeedIn = -0.5;
  private double setSpeed;
  // Vision data
  private double limeLightTx;
  private double limeLightTy;
  private double limeLightA2;
  double h1 = 37.5;
  double h2 = 98.25;
  double a1 = 0.0;
  double kpAim = -0.035;
  boolean targetFound = false;
  boolean isTracking;

  // Turret
  public Turret() {
    
   // final double iaccum = 0;

    turretMotor.setInverted(false);

    /* first choose the sensor- using the CAN Encoder */  
    /*turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Constants.EncoderConstants.kPIDLoopIdx, Constants.EncoderConstants.kTimeoutMs);
    turretMotor.setSensorPhase(true);

    // Reset sensor position
    turretMotor.setIntegralAccumulator(iaccum, 0, 10);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    /*turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.EncoderConstants.kTimeoutMs);
    turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.EncoderConstants.kTimeoutMs);

    /* set the peak and nominal outputs */
    /*turretMotor.configNominalOutputForward(0, Constants.EncoderConstants.kTimeoutMs);
    turretMotor.configNominalOutputReverse(0, Constants.EncoderConstants.kTimeoutMs);
    turretMotor.configPeakOutputForward(1, Constants.EncoderConstants.kTimeoutMs);
    turretMotor.configPeakOutputReverse(-1, Constants.EncoderConstants.kTimeoutMs);

    /* zero the sensor */
    /* turretMotor.setSelectedSensorPosition(0, Constants.EncoderConstants.kPIDLoopIdx, Constants.EncoderConstants.kTimeoutMs); */
  }

  // Periodic
  @Override
  public void periodic() {
    if(isTracking = true) {
    SmartDashboard.putNumber(" TurretSensorPosition", turretMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
    limeLightTx = Robot.m_limeLight.getdegRotationToTarget();
    limeLightTy = Robot.m_limeLight.getdegVerticalToTarget();
    limeLightA2 = Robot.m_limeLight.getdegVerticalToTarget();
    SmartDashboard.putNumber("Vision X", limeLightTx);
    SmartDashboard.putNumber("Vision Y", limeLightTy);
    targetFound = Robot.m_limeLight.getIsTargetFound();
    SmartDashboard.putBoolean("Target Found", targetFound);
    SmartDashboard.putNumber("Estimate Distance", this.EstimatedDistance(limeLightA2));
    } 
  }

  public void TurretHoming() {
    Robot.m_turretVariables.IsMotoredHome = false;
    turretMotor.set(ControlMode.PercentOutput, 0.4);

    //Resets the Encoder Position to Zero / Stop Motor
    if (homeSwitch.get() == true){
      turretMotor.set(ControlMode.PercentOutput, 0.0);
      turretMotor.setSelectedSensorPosition(0);
       this.MoveOffSwitch();
       Robot.m_turretVariables.IsMotoredHome = true;
    }
  }
   // Run
   public void Run() {
    double backup = SpeedIn;
    setSpeed = getPreferencesDouble(TurretSpeed ,backup);
    turretMotor.set(setSpeed);
  }

  public void TrackTarget() {
    isTracking = true;
    if(targetFound == true && homeSwitch.get() == false) {
      setSpeed = limeLightTx * kpAim;
      SmartDashboard.putNumber("Turret Sp", setSpeed);
      turretMotor.set(ControlMode.PercentOutput, setSpeed);
    }
    else {
      setSpeed  = 0.0;
      if(homeSwitch.get() == true) {
        turretMotor.set(ControlMode.PercentOutput, setSpeed);
        this.TrackingMoveOffSwitch();
      }
      else {
        this.stopMotion();
      }
     
    }
  }

  public void StopUpdating() {
    isTracking = false;
  }

  private void TrackingMoveOffSwitch() {
   //looks for home sensor to go off 
    do {
     turretMotor.set(ControlMode.PercentOutput, -0.4);
    } while (homeSwitch.get() == true);
   
  } 

  public void MoveOffHomeSwitch () {
    turretMotor.set(ControlMode.PercentOutput, -0.4);
  }
  
  // Stop Motion 
  public void stopMotion() {
    turretMotor.set(ControlMode.PercentOutput, 0.0);
  }
  
  //---------------------------------------------------------------------
  // Estimated Distance
  //---------------------------------------------------------------------
  private double EstimatedDistance(double a2) {
    return (h2-h1)/Math.tan(a1+a2);
  }

  private void MoveOffSwitch(){
    turretMotor.set(ControlMode.MotionMagic, 0); // put vaule of encoder position
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