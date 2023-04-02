package frc.robot;

//#region [ IMPORTS ]
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutonAutoBalancePIDCommand;
import frc.robot.commands.autonomous.AutonAutoBalancePIDCommandBlue;
import frc.robot.commands.autonomous.AutonDriveTrainMoveCommand;
import frc.robot.commands.autonomous.AutonDriveTrainTurnCommand;
import frc.robot.commands.autonomous.AutonDriveTrainTurnPIDCommand;
import frc.robot.commands.autonomous.AutonElbowPIDControlCommand;
import frc.robot.commands.autonomous.AutonLiftPIDControlCommand;
import frc.robot.commands.autonomous.AutonSlidePIDControlCommand;
import frc.robot.commands.autonomous.MoveToEncoderPitchCommand;
import frc.robot.commands.autonomous.MoveToPitch;
import frc.robot.subsystems.*;
import frc.robot.utils.ExtendedJoystick;
import frc.robot.utils.ExtendedXboxController;
//#endregion

public class RobotContainer {
    // Create joysticks
    private final ExtendedJoystick stick = new ExtendedJoystick(ControllerMapping.JOYSTICK);
    private final ExtendedXboxController controller = new ExtendedXboxController(ControllerMapping.XBOX);

    // Create subsystems
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Claw claw = Claw.getInstance();
    private final Lift lift = Lift.getInstance();
    private final Slide slide = Slide.getInstance();
    private final Elbow elbow = Elbow.getInstance();

    // Create other variables

    ////////////////////////////////
    // #region [ AUTON COMMANDS ]
    // #region HighGoalBackupTurn
    // Auton drops cone/cube and backs up
    private final Command HighGoalBackupTurn = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 225),
                    new AutonSlidePIDControlCommand(slide, -257),
                    new ClawOperateInstantCommand(claw),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            new AutonElbowPIDControlCommand(elbow, -140))),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                    new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonSlidePIDControlCommand(slide, 0),
                    new AutonLiftPIDControlCommand(lift, 0),
                    new AutonDriveTrainMoveCommand(driveTrain, -2850, .5)),
            new WaitCommand(.5),
            new AutonDriveTrainTurnCommand(driveTrain, 540, 1));
    // #endregion

    // #region BalanceOnly
    // Auton moves backwards and balances the robot
    private final Command BalanceOnly = new SequentialCommandGroup(
            new MoveToEncoderPitchCommand(driveTrain, 15, .4),
            new AutonAutoBalancePIDCommand(driveTrain),
            new AutoLockPIDCommand(driveTrain));
    // #endregion
    // #region HighGoalOnly
    // Auton extends and drops a cone/cube on the high goal
    private final Command HighGoalOnly = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 225),
                    new AutonSlidePIDControlCommand(slide, -257),
                    new ClawOperateInstantCommand(claw),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            new AutonElbowPIDControlCommand(elbow, -140))),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(0.4),
            new ParallelCommandGroup(new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonSlidePIDControlCommand(slide, 0),
                    new AutonLiftPIDControlCommand(lift, 0)));
    // #endregion

    // #region LowGoalOnly
    // Auton extends and drops a cone/cube on the low goal
    private final Command LowGoalOnly = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 84),
                    new AutonElbowPIDControlCommand(elbow, -140),
                    new ClawOperateInstantCommand(claw)),
            new WaitCommand(.15),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(.15),
            new ParallelCommandGroup(
                    new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonLiftPIDControlCommand(lift, 0)));
    // #endregion

    // #region LowGoalBalance
    // Auton drops on the low goal, before moving backwards and balancing
    private final Command LowGoalBalance = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 84),
                    new AutonElbowPIDControlCommand(elbow, -140),
                    new ClawOperateInstantCommand(claw)),
            new WaitCommand(.15),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(.15),
            new ParallelCommandGroup(
                    new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonLiftPIDControlCommand(lift, 0)),
            new MoveToPitch(driveTrain, 15, .5),
            new AutonAutoBalancePIDCommand(driveTrain),
            new AutoLockPIDCommand(driveTrain));
    // #endregion

    // #region Muskegon Blue Balance
    // Auton drops on the low goal, before moving backwards and balancing ON THE
    // BLUE SIDE FOR MUSKEGONS WEIRD BALANCE
    private final Command LowGoalBalanceBlue = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 84),
                    new AutonSlidePIDControlCommand(slide, 0),
                    new ClawOperateInstantCommand(claw),
                    new SequentialCommandGroup(
                            new WaitCommand(.5),
                            new AutonElbowPIDControlCommand(elbow, -145))),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(0.4),
            new ParallelCommandGroup(new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonSlidePIDControlCommand(slide, 0),
                    new AutonLiftPIDControlCommand(lift, 0)),
            new MoveToPitch(driveTrain, 15, .5),
            new AutonAutoBalancePIDCommandBlue(driveTrain),
            new AutoLockPIDCommand(driveTrain));
    // #endregion

    // #region HighGoalBalance
    // Scores on the high goal then balances
    private final Command HighGoalBalance = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 225),
                    new AutonSlidePIDControlCommand(slide, -257),
                    new ClawOperateInstantCommand(claw),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            new AutonElbowPIDControlCommand(elbow, -140))),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                    new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonSlidePIDControlCommand(slide, 0),
                    new AutonLiftPIDControlCommand(lift, 0)),
            new MoveToPitch(driveTrain, 15, .5),
            new AutonAutoBalancePIDCommand(driveTrain),
            new AutoLockPIDCommand(driveTrain));
    // #endregion

    // #region ScoreTwice
    // Scores once, picks up, then scores again [WIP]
    private final Command ScoreTwice = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutonLiftPIDControlCommand(lift, 225),
                    new AutonSlidePIDControlCommand(slide, -257),
                    new ClawOperateInstantCommand(claw),
                    new SequentialCommandGroup(
                            new WaitCommand(1),
                            new AutonElbowPIDControlCommand(elbow, -140))),
            new ClawOperateInstantCommand(claw),
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                    new AutonElbowPIDControlCommand(elbow, 0),
                    new AutonSlidePIDControlCommand(slide, 0),
                    new AutonLiftPIDControlCommand(lift, 0),
                    new AutonDriveTrainMoveCommand(driveTrain, -2700, .5)),
            new WaitCommand(.5),
            new AutonDriveTrainTurnCommand(driveTrain, 540, 1),
            new WaitCommand(.6));

    // #endregion

    // #region TurnBot
    // Just turn the bot (for testing)
    private final Command TurnBot = new SequentialCommandGroup(
            // new BrakeOffCommand(driveTrain),
            new AutonDriveTrainTurnCommand(driveTrain, 484, 1),
            new BrakeOnCommand(driveTrain));
    // #endregion

    // #region Backup
    // Just backs up out of the starting area and does nothing else
    private final Command Backup = new SequentialCommandGroup(
            new AutonDriveTrainMoveCommand(driveTrain, -2900, .3));
    // #endregion

    // #endregion
    ////////////////////////////////

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
        driveTrain.resetEncoders();
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

        // Toggle IR sensor
        stick.b_Trigger()
                .onTrue(new ClawAutoIRClose(claw));

        // Toggle Limelight Auto Aim
        stick.b_BottomBL()
                .toggleOnTrue(new LimelightAlignTargetCommand(driveTrain, limelight));

        // Zero out the elbow, slide, and lift
        controller.b_Back()
                .toggleOnTrue(new RezeroAllInstantCommand(elbow, slide, lift));

        // Auto lift to Upper Goal
        controller.b_X()
                .toggleOnTrue(
                        new ParallelCommandGroup(
                                new LiftPIDControlCommand(lift, 225),
                                new SlidePIDControlCommand(slide, -257),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.25),
                                        new ElbowPIDControlCommand(elbow, -120))));

        // Auto lift to Lower Goal
        controller.b_A()
                .toggleOnTrue(
                        new ParallelCommandGroup(
                                new LiftPIDControlCommand(lift, 84),
                                new SlidePIDControlCommand(slide, 0),
                                new ElbowPIDControlCommand(elbow, -145)));

        // Auto lift to substation pickup location
        controller.b_RightStick()
                .toggleOnTrue(
                        new ParallelCommandGroup(
                                new LiftPIDControlCommand(lift, 125),
                                new SlidePIDControlCommand(slide, 0),
                                new ElbowPIDControlCommand(elbow, -150)));

        // Auto retract everything to zero
        controller.b_B().toggleOnTrue(
                new ParallelCommandGroup(
                        new LiftPIDControlCommand(lift, 0),
                        new SlidePIDControlCommand(slide, 0),
                        new ElbowPIDControlCommand(elbow, 0)));

        // Turn drive motor brake on and off
        // controller.b_Back().onTrue(new BrakeOnCommand(driveTrain));
        // controller.b_Start().onTrue(new BrakeOffCommand(driveTrain));

        // Open and close the claw
        // Note: The code knows if it's open or closed and will automaticly do the other
        controller.b_Y().onTrue(new ClawOperateInstantCommand(claw));

    }

    /**
     * Returns the selected autonomous command.
     */
    public Command getAutonomousCommand() {
        // [MAIN AUTONS]
         return HighGoalBackupTurn; // Place piece high, back up, turn 180
        // return HighGoalBalance; // High Goal then balance
        // return LowGoalBalance; // Low goal auto balance

        // [SECONDARY AUTONS]
        // return BalanceOnly; // Backwards auto balance
        // return HighGoalOnly; // High Goal
        // return LowGoalOnly; // Low Goal
        // return Backup; // Backs up out of the zone and nothing else

        // [TESTS DO NOT USE]
        // return LowGoalBalanceBlue; // Muskegon Blue low auto balance
        //return ScoreTwice; // Score, grab another piece and score [WIP]
        // return TurnBot; // Turns the bot for testing [WIP]
    }
}