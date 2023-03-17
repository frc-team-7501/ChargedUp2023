// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonLiftPIDControlCommand extends PIDCommand {
  private final Lift lift;
  private int outputCounter = 0;

  /** Creates a new LiftPIDControlCommand. */
  public AutonLiftPIDControlCommand(final Lift lift, final double position) {
    super(
        // The controller that the command will use
        new PIDController(0.05, 0, 0),
        // This should return the measurement
        () -> lift.getLiftPosition(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          lift.moveLift(output *= 1);
          // SmartDashboard.putNumber("Lift Output", output);
          SmartDashboard.putNumber("Lift", lift.getLiftPosition());
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(lift);
    this.lift = lift;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(15);
    getController().setSetpoint(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.moveLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
    if(getController().atSetpoint()){
      outputCounter = outputCounter + 1;
      //SmartDashboard.putNumber("Output Counter", outputCounter);
      if(outputCounter > 40){
        outputCounter = 0;
        return true;
      } 
      } else {
        outputCounter = 0;
      }
    return false;
  }
}
