package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

public class MecanumConstants {
    public static class WheelConstants {
        public static final double WHEEL_DIAMETER = 0.175;
        public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
    }

    public static class MotorConstants {
        // 1 / 12
        public static double GEAR_RATIO = 12.0;
        // Gear ratio * pi * wheel diamater
        public static final double MAX_RPM = 6000 * GEAR_RATIO;
        public static final double MAX_ANGULAR_VELOCITY_IN_DEGREES = 80;

        // 0.7 mitigates the final value obtained
        public static final double MAX_MPS_DRIVE = 1.5; //(GEAR_RATIO * WheelConstants.WHEEL_RADIUS * Math.PI) / 60;
        public static final double TICKS_PER_REVOLUTION = 28.0;
    }

    public static class ConversionFactors {
        public static final double MOTOR_RPM_TO_METERS = MotorConstants.GEAR_RATIO * Math.PI * WheelConstants.WHEEL_DIAMETER;
        public static final double MOTOR_RPM_TO_METERS_PER_SECOND = MOTOR_RPM_TO_METERS / 60.0;

        public static final double MOTOR_METERS_PER_SECOND_TO_DEGREES = 1 / WheelConstants.WHEEL_RADIUS;
        public static final double MOTOR_TICKS_TO_DEGREES = 1.0 / MotorConstants.TICKS_PER_REVOLUTION * 360;
    }

    public static class IDs {
        public static final String FRONT_LEFT_ID = "frontLeft";
        public static final String FRONT_RIGHT_ID = "frontRight";
        public static final String BACK_LEFT_ID = "backLeft";
        public static final String BACK_RIGHT_ID = "backRight";
    }

    public static class PIDFCoefficients {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 1.2;
    }
}
