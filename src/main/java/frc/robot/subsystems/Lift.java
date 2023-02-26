// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lift extends SubsystemBase {
  private final CANSparkMax liftMotor = new CANSparkMax(CANMapping.SPARKMAX_LIFT, MotorType.kBrushless);

  /** Creates a new Lift. */
  public Lift() {
    liftMotor.restoreFactoryDefaults();
  }

  private static Lift instance;
  public static Lift getInstance() {
    if (instance == null)
      instance = new Lift();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveLift(double speed) {
    liftMotor.set(speed);
  }

  public void stop() {
    liftMotor.stopMotor();
  }
}
