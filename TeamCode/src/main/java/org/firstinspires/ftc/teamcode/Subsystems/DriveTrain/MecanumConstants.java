package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

public class MecanumConstants {
    public static class WheelConstants {
        public static final double WHEEL_DIAMETER = 0.075; // Metros
        public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;

        // 381.05 mm x 343.1 mm
        // distancia entre izquierda y derecha X distancia entre arriba y abajo
        //public static final double WHEEL_BASE =
    }

    public static class MotorConstants {
        public static double GEAR_RATIO = 2.89 * 3.61; // 1 / 12
        // Gear ratio * pi * wheel diamater
        public static final double MAX_RPM = 6000 * GEAR_RATIO;



        // 9-feb-2025 Cambié este también a radianes
        // public static final double MAX_ANGULAR_VELOCITY_IN_DEGREES = MAX_RPM / 60 * (360);
        // el 360 se refiere a la cantidad de grados en una rotación
        // Math.PI * 2 lo convierte a radianes, que es la unidad estándar
        public static final double MAX_ANGULAR_VELOCITY = MAX_RPM / 60 * (Math.PI * 2);



        // 0.7 mitigates the final value obtained
        public static final double MAX_MPS_DRIVE = 1.5; //(GEAR_RATIO * WheelConstants.WHEEL_RADIUS * Math.PI) / 60;
        public static final double TICKS_PER_REVOLUTION = 28.0;
    }

    public static class ConversionFactors {
        /*public static final double MOTOR_RPM_TO_METERS = MotorConstants.GEAR_RATIO * Math.PI * WheelConstants.WHEEL_DIAMETER * (Math.PI/ 180);
        public static final double MOTOR_RPM_TO_METERS_PER_SECOND = MOTOR_RPM_TO_METERS / 60.0;

        public static final double MOTOR_METERS_PER_SECOND_TO_DEGREES = 1 / MOTOR_RPM_TO_METERS; // Asegurarse que estè en m/s*/

        // 9-Feb-2025
        // sacando la angular velocity ——— replazo de la variable MOTOR_METERS_PER_SECOND_TO_DEGREES
                // Formula para obtener la angular velocity:
                // angular velocity = linear velocity / wheel radius
                // rad/s = (m/s) / (meters)
        // andaba leyendo y resulta que es un estándar de industria utilizar radianes antes que grados
        // esto es porque los grados tienden a ser muy volátiles y te dan valores altísimos o bajísimos — no in between
        // entonces mejor cambiarlo a radianes y nos evitamos de conversiones confusas y así
        // en esta variable, dividimos 1 porque nos da una constante que podemos multiplicar por un valor (obtenido
        // desde wheel speeds) para normalizar el valor pasado al motor con .setVelocity()
        public static final double MOTOR_MPS_TO_RADPS = 1 / WheelConstants.WHEEL_RADIUS;
        public static final double MOTOR_TICKS_TO_DEGREES = 360 / MotorConstants.TICKS_PER_REVOLUTION;
    }

    public static class IDs {
        public static final String FRONT_LEFT_ID = "frontLeft";
        public static final String FRONT_RIGHT_ID = "frontRight";
        public static final String BACK_LEFT_ID = "backLeft";
        public static final String BACK_RIGHT_ID = "backRight";
    }


    // 9-Feb-2025 SUPER MODIFIQUÉ LOS PIDS PARA VER SI ESE ES EL TEMA
    // 9-Feb-2025 PREFIERO QUE SALGA DISPARADO, A QUE FUNCIONE PERO NO LO VEAMOS PORQUE LOS PIDS SON BAJOS
    // 9-Feb-2025 nomás les sume 20 a todos
    public static class PIDFCoefficients {
        public static final double kP = 20.0;
        public static final double kI = 20.0;
        public static final double kD = 20.0;
        public static final double kFF = 21.2;
    }
}
