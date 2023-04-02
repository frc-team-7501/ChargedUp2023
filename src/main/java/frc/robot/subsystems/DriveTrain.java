package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANMapping;
import edu.wpi.first.wpilibj.Encoder;

public class DriveTrain extends SubsystemBase {
    private final WPI_VictorSPX motorFL = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_FL);
    private final WPI_VictorSPX motorFR = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_FR);
    private final WPI_VictorSPX motorBL = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_BL);
    private final WPI_VictorSPX motorBR = new WPI_VictorSPX(CANMapping.VICTORSPX_DRIVE_BR);

    private final MotorControllerGroup groupL = new MotorControllerGroup(motorFL, motorBL);
    private final MotorControllerGroup groupR = new MotorControllerGroup(motorFR, motorBR);

    private final Encoder encoderRight = new Encoder(5, 6);
    private final Encoder encoderLeft = new Encoder(7, 8);

    private final DifferentialDrive differentialDrive = new DifferentialDrive(groupL, groupR);

    private final PigeonIMU pigeonIMU = new PigeonIMU(CANMapping.PIGEON_IMU);

    private static DriveTrain instance;

    public static DriveTrain getInstance() {
        if (instance == null)
            instance = new DriveTrain();
        return instance;
    }

    public Boolean getEncoderLeft() {
        return encoderLeft.getStopped();
    }

    private DriveTrain() {
        groupL.setInverted(true);
    }

    public void drive(double forwards, double rotate, boolean quickTurn) {
        if (quickTurn) {
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

    // Returns Left Encoder current distance
    public double getLeftDistance() {
        double LeftD = 0;
        LeftD = encoderLeft.getDistance();
        return LeftD;
    }

    // Returns Right Encoder current distance
    public double getRightDistance() {
        double RightD = 0;
        RightD = encoderRight.getDistance();
        return RightD;
    }

    public void resetEncoders(){
        encoderLeft.reset();
        encoderRight.reset();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Yaw", getGyroYaw());
        SmartDashboard.putNumber("Pitch", getGyroPitch());
        SmartDashboard.putNumber("EncoderR", encoderRight.getDistance());
        SmartDashboard.putNumber("EncoderL", encoderLeft.getDistance());
        if (encoderLeft.getStopped()) {
            SmartDashboard.putString("Left Encoder Status", "OFF");
        } else {
            SmartDashboard.putString("Left Encoder Status", "ON");
        }
    }
}
