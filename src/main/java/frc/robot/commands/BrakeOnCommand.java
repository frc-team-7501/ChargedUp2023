// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BrakeOnCommand extends InstantCommand {
  private final DriveTrain driveTrain;

  public BrakeOnCommand(final DriveTrain driveTrain) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setBrakeMode(true);
  }
}
