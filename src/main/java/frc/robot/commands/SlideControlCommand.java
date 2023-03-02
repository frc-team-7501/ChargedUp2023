// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;
import java.util.function.DoubleSupplier;

public class SlideControlCommand extends CommandBase {
  /** Creates a new SlideControlCommand. */
  private final Slide slide;
  private final DoubleSupplier slideSpeed;
  private double slideSpeedDouble = 0;

  public SlideControlCommand(Slide slidein, DoubleSupplier slideSpeedin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.slide = slidein;
    this.slideSpeed = slideSpeedin;
    addRequirements(slide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    slideSpeedDouble = slideSpeed.getAsDouble() * -1;
    
    // Set to zero to compensate for stick drift
    if (Math.abs(slideSpeedDouble) < 0.05)
    slideSpeedDouble = 0;
    slide.moveSlide(slideSpeedDouble);
    SmartDashboard.putNumber("Slide", slide.getSlidePosition());
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    slide.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
