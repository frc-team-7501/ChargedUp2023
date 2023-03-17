package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutonAutoBalancePIDCommand;
import frc.robot.commands.autonomous.AutonDriveTrainMoveCommand;
import frc.robot.commands.autonomous.AutonDriveTrainTurnPIDCommand;
import frc.robot.commands.autonomous.AutonElbowPIDControlCommand;
import frc.robot.commands.autonomous.AutonLiftPIDControlCommand;
import frc.robot.commands.autonomous.AutonSlidePIDControlCommand;
import frc.robot.commands.autonomous.MoveToPitch;
import frc.robot.subsystems.*;
import frc.robot.utils.ExtendedJoystick;
import frc.robot.utils.ExtendedXboxController;

public class RobotContainer {
        // Create joysticks
        private final ExtendedJoystick stick = new ExtendedJoystick(ControllerMapping.JOYSTICK);
        private final ExtendedXboxController controller = new ExtendedXboxController(ControllerMapping.XBOX);

        // Create subsystems
        private final DriveTrain driveTrain = DriveTrain.getInstance();
        private final Claw claw = Claw.getInstance();
        private final Lift lift = Lift.getInstance();
        private final Slide slide = Slide.getInstance();
        private final Elbow elbow = Elbow.getInstance();

        // Create other variables
        // Autonomous commands
        // Auton drops cone/cube and backs up
        private final Command HighGoalBackupTurn = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                        new AutonLiftPIDControlCommand(lift, 360),
                                        new AutonSlidePIDControlCommand(slide, -257),
                                        new ClawOperateInstantCommand(claw),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(2),
                                                        new AutonElbowPIDControlCommand(elbow, -130))),
                        new ClawOperateInstantCommand(claw),
                        new WaitCommand(0.4),
                        new ParallelCommandGroup(new AutonElbowPIDControlCommand(elbow, 0),
                                        new AutonSlidePIDControlCommand(slide, 0),
                                        new AutonLiftPIDControlCommand(lift, 0),
                                        new AutonDriveTrainMoveCommand(driveTrain, -2700, .5)),
                        new AutonDriveTrainTurnPIDCommand(driveTrain, 180));

        // Auton moves backwards and balances the robot
        private final Command BalanceOnly = new SequentialCommandGroup(
                        new MoveToPitch(driveTrain, 15, .4),
                        new AutonAutoBalancePIDCommand(driveTrain),
                        new AutoLockPIDCommand(driveTrain));

        // Auton extends and drops a cone/cube on the high goal
        private final Command HighGoalOnly = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                        new AutonLiftPIDControlCommand(lift, 360),
                                        new AutonSlidePIDControlCommand(slide, -257),
                                        new ClawOperateInstantCommand(claw),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(2),
                                                        new AutonElbowPIDControlCommand(elbow, -130))),
                        new ClawOperateInstantCommand(claw),
                        new WaitCommand(0.4),
                        new ParallelCommandGroup(new AutonElbowPIDControlCommand(elbow, 0),
                                        new AutonSlidePIDControlCommand(slide, 0),
                                        new AutonLiftPIDControlCommand(lift, 0)));

        // Auton extends and drops a cone/cube on the low goal
        private final Command LowGoalOnly = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                        new AutonLiftPIDControlCommand(lift, 168),
                                        new AutonElbowPIDControlCommand(elbow, -126),
                                        new ClawOperateInstantCommand(claw)),
                        new WaitCommand(.5),
                        new ClawOperateInstantCommand(claw),
                        new WaitCommand(0.2),
                        new ParallelCommandGroup(
                                        new AutonElbowPIDControlCommand(elbow, 0),
                                        new AutonLiftPIDControlCommand(lift, 0)));

        // Auton drops on the low goal, before moving backwards and balancing
        private final Command LowGoalBalance = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                        new AutonLiftPIDControlCommand(lift, 168),
                                        new AutonElbowPIDControlCommand(elbow, -126)),
                        new WaitCommand(1),
                        new ClawOperateInstantCommand(claw),
                        new ClawOperateInstantCommand(claw),
                        new WaitCommand(0.2),
                        new ParallelCommandGroup(
                                        new AutonElbowPIDControlCommand(elbow, 0),
                                        new AutonLiftPIDControlCommand(lift, 0),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(.5),
                                                        new MoveToPitch(driveTrain, 15, .5),
                                                        new AutonAutoBalancePIDCommand(driveTrain),
                                                        new AutoLockPIDCommand(driveTrain))));

        private final Command HighGoalBalance = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                        new AutonLiftPIDControlCommand(lift, 373),
                                        new AutonSlidePIDControlCommand(slide, -257),
                                        new ClawOperateInstantCommand(claw),
                                        new SequentialCommandGroup(
                                                        new WaitCommand(2),
                                                        new AutonElbowPIDControlCommand(elbow, -163))),
                        new ClawOperateInstantCommand(claw),
                        new WaitCommand(0.3),
                        new ParallelCommandGroup(new AutonElbowPIDControlCommand(elbow, 0),
                                        new AutonSlidePIDControlCommand(slide, 0),
                                        new AutonLiftPIDControlCommand(lift, 0)),
                        new MoveToPitch(driveTrain, 15, .4),
                        new AutonAutoBalancePIDCommand(driveTrain),
                        new AutoLockPIDCommand(driveTrain));

        // Create commands
        private final Command driveManualCommand = new DriveManualCommand(
                        driveTrain,
                        () -> stick.getY() * (stick.getThrottle() * 0.5 + 0.5), () -> -stick.getX(),
                        () -> stick.getTop());

        private final Command liftManualCommand = new LiftControlCommand(lift, () -> controller.getLeftY());

        private final Command slideManualCommand = new SlideControlCommand(slide, () -> controller.getLeftX());

        private final Command elbowManualCommand = new ElbowControlCommand(elbow, () -> controller.getRightY());

        public RobotContainer() {

                configureButtonBindings();

                driveTrain.setDefaultCommand(driveManualCommand);
                lift.setDefaultCommand(liftManualCommand);
                slide.setDefaultCommand(slideManualCommand);
                elbow.setDefaultCommand(elbowManualCommand);

                ShuffleboardTab subsysTab = Shuffleboard.getTab("Subsystems");
                subsysTab.add("DriveTrain", driveTrain);
        }

        // Lifecycle hooks

        public void autonomousInit() {
                driveTrain.setBrakeMode(true);
                elbow.resetEncoder();
                lift.resetEncoder();
                slide.resetEncoder();
        }

        public void teleopInit() {
                driveTrain.setBrakeMode(false);
        }

        private void configureButtonBindings() {
                // PID control to auto balance the robot using the PigeonIMU
                controller.b_LeftBumper()
                                .toggleOnTrue(new AutoBalancePIDCommand(driveTrain));

                // PID control to auto lock the robot in place
                // Uses a very low power to not try and climb off the other side but keep in
                // place
                controller.b_RightBumper()
                                .toggleOnTrue(new AutoLockPIDCommand(driveTrain));

                stick.b_Trigger()
                                .onTrue(new ClawAutoIRClose(claw));

                // Auto lift to Upper Goal
                controller.b_X()
                                .toggleOnTrue(
                                                new ParallelCommandGroup(
                                                                new LiftPIDControlCommand(lift, 360),
                                                                new SlidePIDControlCommand(slide, -257),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(2),
                                                                                new ElbowPIDControlCommand(elbow,
                                                                                                -130))));

                // Auto lift to Lower Goal
                controller.b_A()
                                .toggleOnTrue(
                                                new ParallelCommandGroup(
                                                                new LiftPIDControlCommand(lift, 168),
                                                                new SlidePIDControlCommand(slide, 0),
                                                                new ElbowPIDControlCommand(elbow, -145)));

                // Auto lift to substation pickup location
                controller.b_RightStick()
                                .toggleOnTrue(
                                                new ParallelCommandGroup(
                                                                new LiftPIDControlCommand(lift, 250),
                                                                new SlidePIDControlCommand(slide, 0),
                                                                new ElbowPIDControlCommand(elbow, -150)));

                // Auto retract everything to zero
                controller.b_B().toggleOnTrue(
                                new ParallelCommandGroup(
                                                new LiftPIDControlCommand(lift, 0),
                                                new SlidePIDControlCommand(slide, 0),
                                                new ElbowPIDControlCommand(elbow, 0)));

                // Turn drive motor brake on and off
                controller.b_Back().onTrue(new BrakeOnCommand(driveTrain));
                controller.b_Start().onTrue(new BrakeOffCommand(driveTrain));

                // Open and close the claw
                // Note: The code knows if it's open or closed and will automaticly do the other
                controller.b_Y().onTrue(new ClawOperateInstantCommand(claw));

        }

        /**
         * Returns the selected autonomous command.
         */
        public Command getAutonomousCommand() {
                // return HighGoalBackupTurn; // Place piece high, back up, turn 180
                // return BalanceOnly; // Backwards auto balance
                // return HighGoalOnly; // High Goal
                // return LowGoalOnly; // Low Goal
                // return LowGoalBalance; // Low goal auto balance (NOT TESTED)
                 return HighGoalBalance; // High Goal then balance (cannot finish in 15 seconds at this time)
        }
}