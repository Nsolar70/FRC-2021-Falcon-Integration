/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final WPI_VictorSPX climbMotor = new WPI_VictorSPX(Constants.ActuatorConstants.kClimbMotor);
  private final DigitalInput limitSwitchUp = new DigitalInput(Constants.SensorContants.kClimberExtend);
  private final DigitalInput limitSwitchDown = new DigitalInput(Constants.SensorContants.kClimberLower);
  private final Solenoid latch = new Solenoid(0,0);
  boolean isMovingUp;
  boolean isMovingDown;

  public Climber() {
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("limitSwitch1", limitSwitchUp.get());
    //SmartDashboard.putBoolean("limitSwitch2", limitSwitchDown.get());
  }


  //Call by default command in the Robot Container Class
  public void DefaultRun(double upSpeed, double downSpeed) {
    SmartDashboard.putNumber("Up Spd", upSpeed);
    SmartDashboard.putNumber("Down Spd", downSpeed);
    this.ClimbUp(upSpeed);
    this.ClimbDown(downSpeed);
}
private void ClimbUp(double speed) {
    
  if(speed > 0.08) {
    isMovingUp = true;
    double limitSpeed = speed * -0.75;
    
    if(limitSwitchUp.get() == false) {
      latch.set(true);
      climbMotor.set(limitSpeed);
    }
  }
  else {
    isMovingUp = false;
    if (isMovingDown == false) {
      latch.set(false);
      this.StopMotion();
    }
  }
}

private void ClimbDown(double speed) {
  
  if(speed > 0.08) {
    isMovingDown = true;
    double limitSpeed = speed * 0.75;
    
    if(limitSwitchDown.get() == false) {
      latch.set(true);
      climbMotor.set(limitSpeed);
    }
  }
  else {
    isMovingDown = false;
    if(isMovingUp == false) {
      latch.set(false);
      this.StopMotion();
    }
    
  }
}

  public void StopMotion() {
    climbMotor.set(0.0);
  }
}
