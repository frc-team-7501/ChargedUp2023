package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutonAutoBalancePIDCommand;
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
      new MoveToPitch(driveTrain, 12, .4),
      new AutonAutoBalancePIDCommand(driveTrain),
      new AutoLockPIDCommand(driveTrain));

  // Create commands
  private final Command driveManualCommand = new DriveManualCommand(
      driveTrain,
      () -> stick.getY() * (stick.getThrottle() * 0.5 + 0.5), () -> -stick.getX(),
      () -> stick.getTop());

  private final Command liftManualCommand = new LiftControlCommand(lift, () -> controller.getRightY());

  private final Command slideManualCommand = new SlideControlCommand(slide, () -> controller.getLeftY());

  private final Command elbowManualCommand = new ElbowControlCommand(elbow, () -> controller.getRightX());

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
  }

  public void teleopInit() {
    driveTrain.setBrakeMode(false);
  }

  private void configureButtonBindings() {
    controller.b_LeftBumper()
        .toggleOnTrue(new AutoBalancePIDCommand(driveTrain));
    controller.b_RightBumper()
        .toggleOnTrue(new AutoLockPIDCommand(driveTrain));

    controller.b_Back().onTrue(new BrakeOnCommand(driveTrain));
    controller.b_Start().onTrue(new BrakeOffCommand(driveTrain));

    controller.b_Y().onTrue(new ClawOperateInstantCommand(claw));

  }

  /**
   * Returns the selected autonomous command.
   */
  public Command getAutonomousCommand() {
    // TODO: SendableChooser?
    return auton0;
  }
}
