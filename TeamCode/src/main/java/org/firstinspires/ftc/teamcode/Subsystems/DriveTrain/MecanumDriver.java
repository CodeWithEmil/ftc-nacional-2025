package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import androidx.core.util.Pair;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.Supplier;

public class MecanumDriver {
    MecanumDriveTrain mecanumSubsystem;
    IMU imu;
    SparkFunOTOS otos;

    double DEAD_BAND = 0.1;
    Pair<Double, Double> leftJoystick;
    Pair<Double, Double> rightJoystick;

    public MecanumDriver(
            Supplier<Pair<Double, Double>> leftJoystick, Supplier<Pair<Double, Double>> rightJoystick
    ) {
        this.leftJoystick = leftJoystick.get();
        this.rightJoystick = rightJoystick.get();

    }

    public ChassisSpeeds getDesiredSpeeds() {
        double forwardVelocity = leftJoystick.first * MecanumConstants.MotorConstants.MAX_MPS_DRIVE;
        double strafeVelocity = leftJoystick.second * MecanumConstants.MotorConstants.MAX_MPS_DRIVE;
        double angularVelocity = rightJoystick.first * MecanumConstants.MotorConstants.MAX_ANGULAR_VELOCITY_IN_DEGREES; // TODO: get max angular velocity

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardVelocity, strafeVelocity, angularVelocity, getGyroYaw()
        );
    }

    public void apply(MecanumDriveTrain mecanumSubsystem) {
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
        return Rotation2d.fromDegrees(otos.getPosition().h);
    }


    // Setting deadbands for joysticks
    public boolean isLeftWithinDeadband() {
        return Math.abs(leftJoystick.first) > DEAD_BAND || Math.abs(leftJoystick.second) > DEAD_BAND;
    }

    public boolean isRightWithinDeadband() {
        return Math.abs(rightJoystick.first) > DEAD_BAND || Math.abs(rightJoystick.second) > DEAD_BAND;
    }
}