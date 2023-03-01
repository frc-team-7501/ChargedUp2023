// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.MiscMapping;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lift extends SubsystemBase {
  private final CANSparkMax liftMotor = new CANSparkMax(CANMapping.SPARKMAX_LIFT, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private static Lift instance;
  private DigitalInput topLimitSwitch = new DigitalInput(MiscMapping.TOP_LIMIT_SWITCH);
  private DigitalInput bottomLimitSwitch = new DigitalInput(MiscMapping.BOTTOM_LIMIT_SWITCH);
  
  /** Creates a new Lift. */
  public Lift() {
    liftMotor.restoreFactoryDefaults();
    encoder = liftMotor.getEncoder();
  }

  public void resetEncoder() {
    liftMotor.getEncoder().setPosition(0);
  }

  public static Lift getInstance() {
    if (instance == null)
      instance = new Lift();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getLiftPosition() {
    return encoder.getPosition();
  }
  public void moveLift(double speed) {
    liftMotor.set(speed);

     // Stopping lift if limit switch is activated.
    
     if (speed > 0) {
      if (!topLimitSwitch.get()) {
        liftMotor.set(0);

      } else {
        liftMotor.set(speed);
      }
    } else if (speed < 0) {
      if (!bottomLimitSwitch.get()) {
        liftMotor.set(0);
      } else {
        liftMotor.set(speed);
      }
    } else {
      liftMotor.set(0);
    }
    

    //Let controller if limit switch has been reached.
    if (!topLimitSwitch.get() || !bottomLimitSwitch.get()){
      SmartDashboard.putBoolean("LiftLimit", false);
    } else {
      SmartDashboard.putBoolean("LiftLimit", true);
    }
  }

  public void stop() {
    liftMotor.stopMotor();
  }
}
