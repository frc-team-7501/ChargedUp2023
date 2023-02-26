// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import java.util.function.DoubleSupplier;

public class LiftControlCommand extends CommandBase {
  /** Creates a new LiftControlCommand. */
  private final Lift lift;
  private final DoubleSupplier liftSpeed;
  private double liftSpeedDouble = 0;
  private DigitalInput topLimitSwitch = new DigitalInput(0);
  private DigitalInput bottomLimitSwitch = new DigitalInput(1);
  
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
    
    // Set to zero to compensate for stick drift
    if (Math.abs(liftSpeedDouble) < 0.05)
    liftSpeedDouble = 0;

    // Stopping lift if limit switch is activated.
    
    if (liftSpeedDouble > 0) {
      if (!topLimitSwitch.get()) {
        lift.moveLift(0);
      } else {
        lift.moveLift(liftSpeedDouble);
      }
    } else if (liftSpeedDouble < 0) {
      if (!bottomLimitSwitch.get()) {
        lift.moveLift(0);
      } else {
        lift.moveLift(liftSpeedDouble);
      }
    } else {
      lift.moveLift(0);
    }
    

    //Let controller if limit switch has been reached.
    if (!topLimitSwitch.get() || !bottomLimitSwitch.get()){
      SmartDashboard.putBoolean("LiftLimit", false);
    } else {
      SmartDashboard.putBoolean("LiftLimit", true);
    }
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
