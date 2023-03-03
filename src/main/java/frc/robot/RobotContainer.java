package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutonAutoBalancePIDCommand;
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

  // Autonomous commands
  private final Command auton0 = new SequentialCommandGroup(
      new MoveToPitch(driveTrain, 15, .5),
      new AutonAutoBalancePIDCommand(driveTrain),
      new AutoLockPIDCommand(driveTrain));

  private final Command auton1 = new ParallelCommandGroup(
      new LiftPIDControlCommand(lift, 360),
      new SlidePIDControlCommand(slide, -257),
      new SequentialCommandGroup(
          new WaitCommand(2.5),
          new ElbowPIDControlCommand(elbow, -130)),
      new SequentialCommandGroup(
          new WaitCommand(5),
          new ClawOperateInstantCommand(claw),
          new WaitCommand(1),
          new ClawOperateInstantCommand(claw),
          new WaitCommand(0.2)// ,
      // new ElbowPIDControlCommand(elbow, 0),
      // new ParallelCommandGroup(
      // new LiftPIDControlCommand(lift, 0),
      // new SlidePIDControlCommand(slide, 0),
      // new ElbowPIDControlCommand(elbow, 0))
      ));

  private final Command auton2 = new SequentialCommandGroup(
      new ParallelCommandGroup(
          new AutonLiftPIDControlCommand(lift, 360),
          new AutonSlidePIDControlCommand(slide, -257),
          new AutonElbowPIDControlCommand(elbow, -130)),
      new ClawOperateInstantCommand(claw),
      new ClawOperateInstantCommand(claw),
      new WaitCommand(0.2),
      new ParallelCommandGroup(new AutonElbowPIDControlCommand(elbow, 0),
          new AutonSlidePIDControlCommand(slide, 0),
          new AutonLiftPIDControlCommand(lift, 0)));

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
    elbow.resetEncoder();
    lift.resetEncoder();
    slide.resetEncoder();
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

    // Auto lift to Upper Goal
    controller.b_X()
        .toggleOnTrue(
            new ParallelCommandGroup(
                new LiftPIDControlCommand(lift, 360),
                new SlidePIDControlCommand(slide, -257),
                new SequentialCommandGroup(
                    new WaitCommand(2.5),
                    new ElbowPIDControlCommand(elbow, -130))));

    // Auto lift to Lower Goal
    controller.b_A()
        .toggleOnTrue(
            new ParallelCommandGroup(
                new LiftPIDControlCommand(lift, 168),
                new SlidePIDControlCommand(slide, 0),
                new ElbowPIDControlCommand(elbow, -126)));

    // Auto lift to ground pickup location
    controller.b_RightStick()
        .toggleOnTrue(
            new ParallelCommandGroup(
                new LiftPIDControlCommand(lift, 119),
                new SlidePIDControlCommand(slide, 0),
                new ElbowPIDControlCommand(elbow, -250)));

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
    // return auton0;
    // return auton1;
    return auton2;
  }
}
