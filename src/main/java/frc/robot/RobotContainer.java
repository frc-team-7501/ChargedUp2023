package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerMapping;
import frc.robot.commands.DriveManualCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.ExtendedJoystick;
import frc.robot.utils.ExtendedXboxController;
import frc.robot.utils.InputNormalizer;

public class RobotContainer {
  private final ExtendedJoystick stick = new ExtendedJoystick(ControllerMapping.JOYSTICK);
  private final ExtendedXboxController controller = new ExtendedXboxController(ControllerMapping.XBOX);

  private final DriveTrain driveTrain = DriveTrain.getInstance();

  private final Command driveManualCommand = new DriveManualCommand(
    driveTrain,
    () -> stick.getY() * (stick.getThrottle() * 0.5 + 0.5), () -> -stick.getX(),
    () -> stick.getTop()
  );

    public RobotContainer() {

    configureButtonBindings();

    driveTrain.setDefaultCommand(driveManualCommand);
  
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

  }
}
