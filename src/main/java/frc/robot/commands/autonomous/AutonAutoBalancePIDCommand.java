// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonAutoBalancePIDCommand extends PIDCommand {
  private final DriveTrain driveTrain;
  private int outputCounter = 0;

  public AutonAutoBalancePIDCommand(final DriveTrain driveTrain) {
    super(
        // The controller that the command will use
        // p = power, i = increase, d = dampening
        new PIDController(0.435, .008, .03),
        // This should return the measurement
        () -> driveTrain.getGyroPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          driveTrain.drive(output*=.045,0,false);
          //SmartDashboard.putNumber("Output", output*=.045);
        });
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(11);
    getController().setSetpoint(0);
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At Setpoint", getController().atSetpoint());
    if(getController().atSetpoint()){
      outputCounter = outputCounter + 1;
      //SmartDashboard.putNumber("Output Counter", outputCounter);
      if(outputCounter > 100){
        outputCounter = 0;
        return true;
      } else if(!getController().atSetpoint()) {
        outputCounter = 0;
      }
    }
    return false;
  }
}