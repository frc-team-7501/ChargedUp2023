// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class ClawHighInstantCommand extends InstantCommand {
  private final Claw claw;

  public ClawHighInstantCommand(final Claw claw) {
    addRequirements(claw);
    this.claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.ClawCloseHigh();
  }
}
