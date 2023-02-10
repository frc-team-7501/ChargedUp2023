// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PneumaticsMapping;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawCommand extends CommandBase {
  private final Claw claw;
  private int HighLow;

  public ClawCommand(final Claw claw, int HighLow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.claw = claw;
    this.HighLow = HighLow;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.ClawOpen();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("elf", "In ClawCommand");
    if (HighLow == PneumaticsMapping.PNEUMATIC_HIGH) {
      claw.ClawCloseHigh();
      SmartDashboard.putString("elf", "In Claw High");
    } else if (HighLow == PneumaticsMapping.PNEUMATIC_LOW) {
      SmartDashboard.putString("elf", "In Claw Low");
      claw.ClawCloseLow();
    }
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
