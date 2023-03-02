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

public class Slide extends SubsystemBase {
  private final CANSparkMax slideMotor = new CANSparkMax(CANMapping.SPARKMAX_SLIDE, MotorType.kBrushless);
  private DigitalInput outLimitSwitch = new DigitalInput(MiscMapping.OUT_LIMIT_SWITCH);
  private DigitalInput inLimitSwitch = new DigitalInput(MiscMapping.IN_LIMIT_SWITCH);
  private RelativeEncoder encoder;

  /** Creates a new Slide. */
  public Slide() {
    slideMotor.restoreFactoryDefaults();
    encoder = slideMotor.getEncoder();
  }

  private static Slide instance;

  public static Slide getInstance() {
    if (instance == null)
      instance = new Slide();
    return instance;
  }

  public void resetEncoder() {
    slideMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSlidePosition() {
    return encoder.getPosition();
  }

  public void moveSlide(double speed) {
    // Stopping slide if limit switch is activated
    if (speed < 0) {
      if (!outLimitSwitch.get()) {
        slideMotor.set(0);
      } else {
        slideMotor.set(speed);
      }
    } else if (speed > 0) {
      if (!inLimitSwitch.get()) {
        slideMotor.set(0);
      } else {
        slideMotor.set(speed);
      }
    } else {
      slideMotor.set(0);
    }

    // Let controller if limit switch has been reached.
    if (!outLimitSwitch.get() || !inLimitSwitch.get()) {
      SmartDashboard.putBoolean("SlideLimit", false);
    } else {
      SmartDashboard.putBoolean("SlideLimit", true);
    }
  }

  public void stop() {
    slideMotor.stopMotor();
  }
}
