package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANMapping;

public class DriveTrain extends SubsystemBase {
  private final WPI_VictorSPX motorFL = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_FL);
  private final WPI_VictorSPX motorFR = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_FR);
  private final WPI_VictorSPX motorBL = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_BL);
  private final WPI_VictorSPX motorBR = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_BR);
  
  private final MotorControllerGroup groupL = new MotorControllerGroup(motorFL, motorBL);
  private final MotorControllerGroup groupR = new MotorControllerGroup(motorFR, motorBR);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(groupL, groupR);

  private final PigeonIMU pigeonIMU = new PigeonIMU(Constants.CANMapping.pigeonIMU);

  private static DriveTrain instance;
  public static DriveTrain getInstance() {
    if (instance == null)
      instance = new DriveTrain();
    return instance;
  }

  
  private DriveTrain() {
    groupL.setInverted(true);
  }

  public void drive(double forwards, double rotate, boolean quickTurn) {
    if (quickTurn){
      rotate *= .5;
    }
    differentialDrive.curvatureDrive(forwards, -rotate, quickTurn);
  }

  public void setBrakeMode(boolean enabled) {
    motorBL.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    motorBR.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    motorFL.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    motorFR.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void stop() {
    drive(0, 0, false);
  }

  public double getGyroYaw() {
    double[] ypr = new double[3];
    pigeonIMU.getYawPitchRoll(ypr);
    return ypr[0]; 
  }

  public double getGyroPitch() {
    double[] ypr = new double[3];
    pigeonIMU.getYawPitchRoll(ypr);
    return ypr[1]; 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", getGyroYaw());
    SmartDashboard.putNumber("Pitch", getGyroPitch());
    // SmartDashboard.putNumber("EncoderR", encoderRight.getDistance());
    // SmartDashboard.putNumber("EncoderL", encoderLeft.getDistance());
  }
}
