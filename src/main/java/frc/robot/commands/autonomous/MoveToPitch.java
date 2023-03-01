// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class MoveToPitch extends CommandBase {
  private final DriveTrain driveTrain;
  private final double targetPitch;
  private final double roboSpeed;
  private boolean onTarget = false;

  /** Creates a new MoveToPitch. */
  public MoveToPitch(final DriveTrain driveTrain, final double targetPitch, final double roboSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.targetPitch = targetPitch;
    this.roboSpeed = roboSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // === DO NOT REMOVE ===
    //SmartDashboard.putString("String", "sting :)");
    double robotPitch = driveTrain.getGyroPitch();
    if (Math.abs(robotPitch) < targetPitch) {
      driveTrain.drive(roboSpeed, 0 , false);
    } else {
      driveTrain.drive(0,0 , false);
      onTarget = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (onTarget){
      onTarget = false;
      return true;
    }
    return false;
  }
}
