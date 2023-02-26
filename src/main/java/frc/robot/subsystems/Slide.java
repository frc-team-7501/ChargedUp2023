// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Slide extends SubsystemBase {
 private final CANSparkMax slideMotor = new CANSparkMax(CANMapping.SPARKMAX_SLIDE, MotorType.kBrushless);

  /** Creates a new Slide. */
  public Slide() {
    slideMotor.restoreFactoryDefaults();
  }

  private static Slide instance;
  public static Slide getInstance() {
    if (instance == null)
      instance = new Slide();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveSlide(double speed) {
    slideMotor.set(speed);
  }

  public void stop() {
    slideMotor.stopMotor();
  }
}
