package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import androidx.core.util.Pair;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

public class MecanumDriver {
    public IMU imu;
    public SparkFunOTOS otos;
    Telemetry telemetry;

    double DEAD_BAND = 0.1;
    private final Supplier<Pair<Double, Double>> leftJoystickSupplier;
    private final Supplier<Pair<Double, Double>> rightJoystickSupplier;

    public MecanumDriver(
            Supplier<Pair<Double, Double>> leftJoystick, Supplier<Pair<Double, Double>> rightJoystick,
            HardwareMap hardwareMap, Telemetry telemetry
    ) {
        // Acá también cambió:
        // Antes: this.leftJoystick = leftJoystick.get();
        // Antes: this.rightJoystick = rightJoystick.get();

        // Entonces, se guarda LA FUNCIÓN dentro de nuestro supplier, no los valores como tal
        // Esto permite que podamos actualizar el valor de las palancas (y por lo tanto del supplier)
        // en tiempo real, y así, hacer que avance el robot :)
        this.leftJoystickSupplier = leftJoystick;
        this.rightJoystickSupplier = rightJoystick;

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        //otos.resetTracking();

        imu = hardwareMap.get(IMU.class, "imu");
        //imu.resetYaw();

        this.telemetry = telemetry;
    }

    public ChassisSpeeds getDesiredSpeeds() {
        // Acá también cambió:
        // Así, cada vez que getDesiredSpeeds() sea llamado, obtiene los valores más recientes
        // de los joysticks utilizando leftJoystickSupplier.get()
        Pair<Double, Double> leftJoystick = leftJoystickSupplier.get();
        Pair<Double, Double> rightJoystick = rightJoystickSupplier.get();

        double forwardVelocity = leftJoystick.first * MecanumConstants.MotorConstants.MAX_MPS_DRIVE;
        double strafeVelocity = leftJoystick.second * MecanumConstants.MotorConstants.MAX_MPS_DRIVE;
        double angularVelocity = rightJoystick.first * MecanumConstants.MotorConstants.MAX_ANGULAR_VELOCITY; // * 0.8

        // Telemetría
        /*telemetry.addData("leftX", leftJoystick.first);
        telemetry.addData("leftY", leftJoystick.second);
        telemetry.addData("rightX", rightJoystick.first);
        telemetry.addData("rightY", rightJoystick.second);
        telemetry.addData("forwardVelocity", forwardVelocity);
        telemetry.addData("strafeVelocity", strafeVelocity);
        telemetry.addData("angularVelocity", angularVelocity);
        telemetry.update();*/

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardVelocity, strafeVelocity, angularVelocity, getGyroYaw()
        );
    }

    public void apply(MecanumDriveTrain mecanumSubsystem) {
        telemetry.addData("Apply Driver", true);
        mecanumSubsystem.drive(getDesiredSpeeds());
    }

    public SparkFunOTOS getOTOS() {
        return otos;
    }

    public IMU getIMU() {
        return imu;
    }

    public Rotation2d getGyroYaw() {
        // Get the current position from the gyro sensor
        // Se tiene que negar el valor del Otos para que las posiciones del joystick sean interpretadas
        // correctamente cuando el robot está en posición perpendicular a la de inicio (a 90º)
        return Rotation2d.fromDegrees(-otos.getPosition().h); // orientado a la cancha
        // orientado al robot: return Rotation2d.fromDegrees(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }





    // TODO: Setting deadbands for joysticks
    public boolean isLeftWithinDeadband() {
        return Math.abs(leftJoystickSupplier.get().first) > DEAD_BAND || Math.abs(leftJoystickSupplier.get().second) > DEAD_BAND;
    }

    public boolean isRightWithinDeadband() {
        return Math.abs(rightJoystickSupplier.get().first) > DEAD_BAND || Math.abs(rightJoystickSupplier.get().second) > DEAD_BAND;
    }
}