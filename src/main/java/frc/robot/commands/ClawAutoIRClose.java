// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MiscMapping;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawAutoIRClose extends CommandBase {
  /** Creates a new ClawAutoIRClose. */
  private final Claw claw;
  private DigitalInput irTriggerSwitch = new DigitalInput(MiscMapping.IR_TRIGGER_SWITCH);

  public ClawAutoIRClose(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
if (!irTriggerSwitch.get()) {
        claw.ClawIRClose();
        SmartDashboard.putString("irTrigger", "ON");
      } else {
        SmartDashboard.putString("irTrigger", "OFF");
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
