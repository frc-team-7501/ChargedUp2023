// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Slide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlidePIDControlCommand extends PIDCommand {
  private final Slide slide;
  
  /** Creates a new SlidePIDControlCommand. */
  public SlidePIDControlCommand(final Slide slide, final double position) {
    super(
        // The controller that the command will use
        new PIDController(0.04, 0, 0),
        // This should return the measurement
        () -> slide.getSlidePosition(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          slide.moveSlide(output*=.2);
          //SmartDashboard.putNumber("Slide Output", output);
          SmartDashboard.putNumber("Slide",slide.getSlidePosition());
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(slide);
    this.slide = slide;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(15);
    getController().setSetpoint(0);

  }

  @Override
  public void end(boolean interrupted) {
    slide.moveSlide(0);;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}