// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elbow extends SubsystemBase {
  private final CANSparkMax elbowMotor = new CANSparkMax(CANMapping.SPARKMAX_ELBOW, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private static Elbow instance;

  /** Creates a new Elbow. */
  public Elbow() {
    elbowMotor.restoreFactoryDefaults();
    encoder = elbowMotor.getEncoder();
  }

  public static Elbow getInstance() {
    if (instance == null)
    instance = new Elbow();
    return instance;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getElbowPosition() {
    return encoder.getPosition();
  }

  public void moveElbow(double speed) {
    elbowMotor.set(speed);
  }

  public void stop() {
    elbowMotor.stopMotor();
  }
}
