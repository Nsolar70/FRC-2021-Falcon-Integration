
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderConstants;

public class DriveTrain extends SubsystemBase {
  
  // Drive Motors
   private final WPI_TalonFX frontLeftMotor = new WPI_TalonFX(DriveConstants.kFrontLeftMotor);
   private final WPI_TalonFX backLeftMotor = new WPI_TalonFX(DriveConstants.kBackLeftMotor);
   private final WPI_TalonFX frontRightMotor = new WPI_TalonFX(DriveConstants.kFrontRightMotor);
   private final WPI_TalonFX backRightMotor = new WPI_TalonFX(DriveConstants.kBackRightMotor);
  
  double joyThreshold = 0.05; // Default threshold value from XboxController
  
  // Robot Drive
   MecanumDrive m_drive = new MecanumDrive(frontLeftMotor, backLeftMotor,frontRightMotor, backRightMotor);

  // Array for drive motors
   private  static final int kMaxNumberOfMotors = 4;
   private WPI_TalonFX[] m_TalonFXs = new WPI_TalonFX[kMaxNumberOfMotors];

  // Drive Train
  public DriveTrain() {

    // add motors to array
    m_TalonFXs[1] = frontLeftMotor;
    m_TalonFXs[2] = backLeftMotor;
    m_TalonFXs[3] = frontRightMotor;
    m_TalonFXs[4] = backRightMotor;

     final double iaccum = 0;

    // on / off for motor saftey mode
     m_drive.setSafetyEnabled(false);

     int talonIndex = 0;

    // Falcon Set Up
    for(talonIndex = 0; talonIndex< kMaxNumberOfMotors; talonIndex++){
        // Reset to default to remove any errors associated
        m_TalonFXs[talonIndex].configFactoryDefault();
        //Current Limiting
        m_TalonFXs[talonIndex].configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
        m_TalonFXs[talonIndex].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
        // Encoder Setup
        m_TalonFXs[talonIndex].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
        m_TalonFXs[talonIndex].setIntegralAccumulator(iaccum, 0, 10);
        m_TalonFXs[talonIndex].setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.EncoderConstants.kTimeoutMs);
        m_TalonFXs[talonIndex].setSelectedSensorPosition(0, Constants.EncoderConstants.kPIDLoopIdx, Constants.EncoderConstants.kTimeoutMs); // Zero the sensors
      
        m_TalonFXs[talonIndex].configNominalOutputForward(0, Constants.EncoderConstants.kTimeoutMs);
        m_TalonFXs[talonIndex].configNominalOutputReverse(0, Constants.EncoderConstants.kTimeoutMs);
        m_TalonFXs[talonIndex].configPeakOutputForward(1, Constants.EncoderConstants.kTimeoutMs);
        m_TalonFXs[talonIndex].configPeakOutputReverse(-1, Constants.EncoderConstants.kTimeoutMs);
    }     
  }

  // Periodic
  @Override
  public void periodic() {

   // Pushing Drive Encoder Data to the SmartDashboard
   SmartDashboard.putNumber(" FrontLeftSensorPosition", frontLeftMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
   SmartDashboard.putNumber(" BackLeftSensorPosition", backLeftMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
   SmartDashboard.putNumber(" FrontRightSensorPosition", frontRightMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
   SmartDashboard.putNumber(" BackRightSensorPosition", backRightMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
  }
  
  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * 
   */
  public void drive(double xSpeed, double ySpeed, double rot ) {
    
    if(Math.abs(xSpeed) > joyThreshold || Math.abs(ySpeed) > joyThreshold || Math.abs(rot) > joyThreshold ) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, 0.0 );

      m_drive.driveCartesian(ySpeed*-1.0, xSpeed*1.0, rot*-0.6);
    }
    else {
      m_drive.driveCartesian(0.0, 0.0, 0.0);
    }
  }

  public void autoDriveBackwards(){
    frontLeftMotor.set(ControlMode.PercentOutput, 1.0);
    backLeftMotor.set(ControlMode.PercentOutput, 1.0);
    frontRightMotor.set(ControlMode.PercentOutput, -1.0);
    backRightMotor.set(ControlMode.PercentOutput, -1.0); 
  }

  public void autoDriveForward(){
    frontLeftMotor.set(ControlMode.PercentOutput, -1.0);
    backLeftMotor.set(ControlMode.PercentOutput, -1.0);
    frontRightMotor.set(ControlMode.PercentOutput, 1.0);
    backRightMotor.set(ControlMode.PercentOutput, 1.0); 
  }

  public void stopMotion() {
    frontLeftMotor.set(ControlMode.PercentOutput, 0.0);
    backLeftMotor.set(ControlMode.PercentOutput, 0.0);
    frontRightMotor.set(ControlMode.PercentOutput, 0.0);
    backLeftMotor.set(ControlMode.PercentOutput, 0.0);
  }
  }

