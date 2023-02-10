// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsMapping;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase {
  private final Solenoid airSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_AIR);
  private final Solenoid highSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_HIGH);
  private final Solenoid lowSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_LOW);

  private static Claw instance;

  public static Claw getInstance() {
    if (instance == null)
      instance = new Claw();
    return instance;
  }

  public Claw() {
  }

  private int bean = 0;
  public void ClawOpen() {
    SmartDashboard.putNumber("Counter Open", bean);
    SmartDashboard.putString("Claw", "Open");
    bean++;
    airSolenoid.set(false);
    highSolenoid.set(false);
    lowSolenoid.set(false);
  }

  public void ClawCloseHigh() {
    SmartDashboard.putString("Claw", "High");
    if (airSolenoid.get()) {
      ClawOpen();
    } else {
      airSolenoid.set(true);
      lowSolenoid.set(false);
      highSolenoid.set(true);
    }
  }

  public void ClawCloseLow() {
    SmartDashboard.putString("Claw", "Low");
    if (airSolenoid.get()) {
      ClawOpen();
    } else {
      airSolenoid.set(true);
      highSolenoid.set(false);
      lowSolenoid.set(true);
    }
  }

  public void stop() {
    ClawOpen();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("AirSolenoid", airSolenoid.get());
    SmartDashboard.putBoolean("HighSolenoid", highSolenoid.get());
    SmartDashboard.putBoolean("LowSolenoid", lowSolenoid.get());
  }
}
