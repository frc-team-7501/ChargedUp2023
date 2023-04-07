// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.SimpleControllerSlow;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDriveTrainMoveCommandCOPY extends CommandBase {

    SimpleControllerSlow controller;
    private final DriveTrain driveTrain;
    private double relativeSetpoint = 0;
    private double maxSpeed;
    private double skippy = 0.56;
    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;

    public AutonDriveTrainMoveCommandCOPY(DriveTrain driveTrain, double setpoint, double maxSpeed) {
        this.driveTrain = driveTrain;
        this.relativeSetpoint = setpoint;
        this.maxSpeed = maxSpeed;

        controller = new SimpleControllerSlow(0.01, 0.05, driveTrain::getLeftDistance,
                (output) -> driveTrain.drive(output, skippy, true), maxSpeed);

        addRequirements(driveTrain);
        controller.setTolerance(5);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(driveTrain.getLeftDistance() + relativeSetpoint);
        controller.enable();
    }

    @Override
    public void execute() {
        double leftDistance = driveTrain.getLeftDistance();
        double rightDistance = driveTrain.getRightDistance();

        double leftDistanceRate = leftDistance - lastLeftDistance;
        double rightDistanceRate = rightDistance - lastRightDistance;

        lastLeftDistance = leftDistance;
        lastRightDistance = rightDistance;

        if (Math.abs(leftDistanceRate) > Math.abs(rightDistanceRate)) {
            skippy += -.0001;
        } else if (Math.abs(leftDistanceRate) < Math.abs(rightDistanceRate)) {
            skippy += .0001;
        }
        controller.execute();
    }

    @Override
    public void end(boolean interrupted) {
        controller.disable();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
