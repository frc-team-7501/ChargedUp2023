// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class ElbowControlCommand extends CommandBase {
  /** Creates a new ElbowControlCommand. */
  private final Elbow elbow;
  private final DoubleSupplier elbowSpeed;
  private double elbowSpeedDouble = 0;
  
  public ElbowControlCommand(Elbow elbowin, DoubleSupplier elbowSpeedin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elbow = elbowin;
    this.elbowSpeed = elbowSpeedin;
    addRequirements(elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //invert and slow down elbow motor speed.
    elbowSpeedDouble = elbowSpeed.getAsDouble() * -1;
    elbowSpeedDouble = elbowSpeedDouble * 0.5;
    
    // Set to zero to compensate for stick drift.
    if (Math.abs(elbowSpeedDouble) < 0.05)
    elbowSpeedDouble = 0;

    elbow.moveElbow(elbowSpeedDouble);
    SmartDashboard.putNumber("Elbow",elbow.getElbowPosition());
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
   elbow.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
