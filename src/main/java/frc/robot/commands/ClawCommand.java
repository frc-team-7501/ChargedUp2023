// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PneumaticsMapping;
import frc.robot.subsystems.Claw;

public class ClawCommand extends CommandBase {
  private final Claw claw;

  public ClawCommand(final Claw claw, int HighLow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.claw = claw;

    if (HighLow == PneumaticsMapping.PNEUMATIC_HIGH) {
      claw.ClawCloseHigh();
    } else if (HighLow == PneumaticsMapping.PNEUMATIC_LOW) {
      claw.ClawCloseLow();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.ClawOpen();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
