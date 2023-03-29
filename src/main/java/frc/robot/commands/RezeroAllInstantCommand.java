// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Slide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RezeroAllInstantCommand extends InstantCommand {
  private final Elbow elbow;
  private final Slide slide;
  private final Lift lift;

  public RezeroAllInstantCommand(final Elbow elbow, final Slide slide, final Lift lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elbow, slide, lift);
    this.elbow = elbow;
    this.slide = slide;
    this.lift = lift;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbow.resetEncoder();
    lift.resetEncoder();
    slide.resetEncoder();
  }
}
