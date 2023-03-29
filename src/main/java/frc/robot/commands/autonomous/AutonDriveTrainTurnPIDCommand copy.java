// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.MathUtil;

public class AutonDriveTrainTurnPIDCommand extends PIDCommand {
  private final DriveTrain driveTrain;
  private double relativeSetpoint = 0;

private static double normalizedAngle(double angle) {
  return ((angle % 360) + 360) % 360;
}

  public AutonDriveTrainTurnPIDCommand(DriveTrain driveTrain, double setpoint) {
    super(
      // PID controller
      new PIDController(0.031, 0.003, 0.03),
      // Measurement
      () -> normalizedAngle(driveTrain.getGyroYaw()), 
      // The PID setpoint (0 so we can center the bot)
      0,
      // Output consumer
      // output -> driveTrain.curvatureDrive(0, -output, true)
      output -> {
        double clamped = MathUtil.clamp(output, -0.7, 0.7);
        //SmartDashboard.putNumber("clamped", clamped);
        //SmartDashboard.putNumber("output", output);
        //SmartDashboard.putNumber("Yaw", driveTrain.getGyroYaw());
        //SmartDashboard.putNumber("Yaw Normal", normalizedAngle(driveTrain.getGyroYaw()));
        driveTrain.drive(0, clamped, true);
      }
    );

getController().setIntegratorRange(-0.5, 0.5);
getController().enableContinuousInput(0, 360);

    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.relativeSetpoint = setpoint;

    // Tune the PID
    getController().setTolerance(1, 15);
  }

  @Override
  public void initialize() {
    super.initialize();
    getController().setSetpoint(normalizedAngle(driveTrain.getGyroYaw() + relativeSetpoint));
    //SmartDashboard.putNumber("setpoint", normalizedAngle(driveTrain.getGyroYaw() + relativeSetpoint));
  }

@Override
public void execute() {
  m_useOutput.accept(m_controller.calculate(m_measurement.getAsDouble(), m_controller.getSetpoint()));
}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
