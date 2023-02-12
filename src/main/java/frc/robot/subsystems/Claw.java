// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsMapping;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase {
  private final Solenoid highLowSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_AIR);
  private final Solenoid extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_HIGH);
  private final Solenoid retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_LOW);

  private static Claw instance;

  public static Claw getInstance() {
    if (instance == null)
      instance = new Claw();
    return instance;
  }

  public Claw() {
  }

  public void ClawOpen() {
    // Schedules code to be delivered without disrupting other code
    final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

    highLowSolenoid.set(false);
    extendSolenoid.set(false);
    retractSolenoid.set(true);

    // Waits a second without stopping any other code
    final Runnable retract = new Runnable(){
      public void run(){
        retractSolenoid.set(false);
      }
    };
    executorService.schedule(retract, 1000, TimeUnit.MILLISECONDS);
  }

  public void ClawCloseHigh() {
    if (highLowSolenoid.get()) {
      ClawOpen();
    } else {
      highLowSolenoid.set(true);
      retractSolenoid.set(false);
      extendSolenoid.set(true);
    }
  }

  public void ClawCloseLow() {
    if (highLowSolenoid.get()) {
      ClawOpen();
    } else {
      highLowSolenoid.set(true);
      extendSolenoid.set(true);
      retractSolenoid.set(false);
    }
  }

  public void stop() {
    ClawOpen();
  }

  @Override
  public void periodic() {
    // Periodic is called every 20ms, updating our SmartDashboard
    if(highLowSolenoid.get()){
      SmartDashboard.putString("High/Low", "High");
    } else{
      SmartDashboard.putString("High/Low", "Low");
    }

    if(extendSolenoid.get()){
      SmartDashboard.putString("Extend/Retract", "Extend");
    } else{
      SmartDashboard.putString("Extend/Retract", "Retract");
    }
    
  }
}
