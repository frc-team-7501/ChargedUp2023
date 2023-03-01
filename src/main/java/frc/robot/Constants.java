package frc.robot;

public final class Constants {
    public static final class CANMapping {

        // DriveTrain 2023 Bot
        public static final int VICTORSPX_DRIVE_BL = 2;
        public static final int VICTORSPX_DRIVE_BR = 4;
        public static final int VICTORSPX_DRIVE_FL = 1;
        public static final int VICTORSPX_DRIVE_FR = 3;

        // DriveTrain Geargle
        /*
         * public static final int VICTORSPX_DRIVE_BL = 3;
         * public static final int VICTORSPX_DRIVE_BR = 1;
         * public static final int VICTORSPX_DRIVE_FL = 4;
         * public static final int VICTORSPX_DRIVE_FR = 2;
         */

        // Lift, slide and elbow
        public static final int SPARKMAX_LIFT = 41;
        public static final int SPARKMAX_SLIDE = 42;
        public static final int SPARKMAX_ELBOW = 43;

        // MISC CAN Bus
        public static final int PIGEON_IMU = 20;
    }

    public static final class ControllerMapping {
        public static final int JOYSTICK = 0;
        public static final int XBOX = 1;
    }

    public static final class PneumaticsMapping {
        // Pneumatics Control Module
        public static final int PNEUMATIC_SINGLE_SOLENOID_EXTEND = 1;
        public static final int PNEUMATIC_SINGLE_SOLENOID_RETRACT = 2;
    }

    public static final class MiscMapping {
        public static final boolean BRAKE_ON = true;
        public static final boolean BRAKE_OFF = false;
        public static final int TOP_LIMIT_SWITCH = 0;
        public static final int BOTTOM_LIMIT_SWITCH = 1;
        public static final int OUT_LIMIT_SWITCH = 2;
        public static final int IN_LIMIT_SWITCH = 3;
    }
}
