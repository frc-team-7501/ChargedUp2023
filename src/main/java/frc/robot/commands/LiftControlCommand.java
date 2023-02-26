// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class LiftControlCommand extends CommandBase {
  /** Creates a new LiftControlCommand. */
  private final Lift lift;
  private final DoubleSupplier liftSpeed;
  private double liftSpeedDouble = 0;
  
  public LiftControlCommand(Lift liftin, DoubleSupplier liftSpeedin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lift = liftin;
    this.liftSpeed = liftSpeedin;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    liftSpeedDouble = liftSpeed.getAsDouble() * -1;
    SmartDashboard.putNumber("LiftBefore",liftSpeedDouble);
    
    // Set to zero to compensate for stick drift
    if (Math.abs(liftSpeedDouble) < 0.05)
    liftSpeedDouble = 0;

    lift.moveLift(liftSpeedDouble);
    SmartDashboard.putNumber("Lift", liftSpeedDouble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
