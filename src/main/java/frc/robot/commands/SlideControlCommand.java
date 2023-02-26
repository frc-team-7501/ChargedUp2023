// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class SlideControlCommand extends CommandBase {
  /** Creates a new SlideControlCommand. */
  private final Slide slide;
  private final DoubleSupplier slideSpeed;
  private double slideSpeedDouble = 0;
  private DigitalInput outLimitSwitch = new DigitalInput(2);
  private DigitalInput inLimitSwitch = new DigitalInput(3);

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

    // Stopping slide if limit switch is activated
    SmartDashboard.putBoolean("OLS", outLimitSwitch.get());
    SmartDashboard.putNumber("SlideSpeed", slideSpeedDouble);
    if (slideSpeedDouble < 0) {
      if (!outLimitSwitch.get()) {
        slide.moveSlide(0);
      } else {
        slide.moveSlide(slideSpeedDouble);
      }
    } else if (slideSpeedDouble > 0) {
      if (!inLimitSwitch.get()) {
        slide.moveSlide(0);
      } else {
        slide.moveSlide(slideSpeedDouble);
      }
    } else {
      slide.moveSlide(0);
    }
  

  //Let controller if limit switch has been reached.
  if (!outLimitSwitch.get() || !inLimitSwitch.get()){
    SmartDashboard.putBoolean("SlideLimit", false);
  } else {
    SmartDashboard.putBoolean("SlideLimit", true);
  }
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
