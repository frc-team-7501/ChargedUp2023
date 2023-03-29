package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LEDState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class LimelightAlignTargetCommand extends PIDCommand {
  private final Limelight limelight;
  
  public LimelightAlignTargetCommand(DriveTrain driveTrain, Limelight limelight) {
    super(
      // PID controller
      new PIDController(0.075, 0.05, 0),
      // Measurement
      () -> limelight.validTarget() ? limelight.getNormalXOffset() - 10.5 : 0.0,
      // The PID setpoint (0 so we can center the bot)
      () -> 0,
      // Output consumer
      output -> driveTrain.drive(0, output, true)
    );

    this.limelight = limelight;
    addRequirements(driveTrain, limelight);

    // Tune the PID
    getController().setTolerance(0.01);
  }

  @Override
  public void initialize() {
    super.initialize();
    limelight.setLEDState(LEDState.kSolid);
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setLEDState(LEDState.kOff);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
