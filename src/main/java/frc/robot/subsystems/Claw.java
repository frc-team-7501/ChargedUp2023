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
import frc.robot.Constants.CANMapping;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase {
  private final Solenoid extendSolenoid = new Solenoid(CANMapping.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_EXTEND);
  private final Solenoid retractSolenoid = new Solenoid(CANMapping.PNEUMATIC_HUB,PneumaticsModuleType.REVPH,
      PneumaticsMapping.PNEUMATIC_SINGLE_SOLENOID_RETRACT);

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

  public void ClawClose() {
    if (extendSolenoid.get()) {
      ClawOpen();
    } else {
      retractSolenoid.set(false);
      extendSolenoid.set(true);
    }
  }

  public void ClawIRClose() {
      retractSolenoid.set(false);
      extendSolenoid.set(true);
  }

  public void stop() {
    ClawOpen();
  }

  @Override
  public void periodic() {
    // Periodic is called every 20ms, updating our SmartDashboard
    if(extendSolenoid.get()){
      SmartDashboard.putString("Extend/Retract", "Extend/Closed");
    } else{
      SmartDashboard.putString("Extend/Retract", "Retract/Open");
    }
    
  }
}
