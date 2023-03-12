// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Elbow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonElbowPIDControlCommand extends PIDCommand {
  private final Elbow elbow;
  private int outputCounter = 0;

  /** Creates a new ElbowPIDControlCommand. */
  public AutonElbowPIDControlCommand(final Elbow elbow, final double position) {
    super(
        // The controller that the command will use
        new PIDController(0.042, 0.007501, 0.01337),
        // This should return the measurement
        () -> elbow.getElbowPosition(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          elbow.moveElbow(output*=.7);
          SmartDashboard.putNumber("Elbow",elbow.getElbowPosition());
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elbow);
    this.elbow = elbow;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(15);
    getController().setSetpoint(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elbow.moveElbow(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
    if(getController().atSetpoint()){
      outputCounter = outputCounter + 1;
      //SmartDashboard.putNumber("Output Counter", outputCounter);
      if(outputCounter > 25){
        outputCounter = 0;
        return true;
      } 
      } else {
        outputCounter = 0;
    }
    return false;
  }
}
