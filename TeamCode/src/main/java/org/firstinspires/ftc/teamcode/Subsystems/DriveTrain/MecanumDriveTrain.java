package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants.IDs;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants.ConversionFactors;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants.PIDFCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;


import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveTrain extends SubsystemBase {

    // Declaring telemetry
    private Telemetry telemetry;


    // Declaring motors
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;


    // ! Kinematics
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            // In this order: frontLeft, frontRight, backLeft, backRight
            new Translation2d(0.381, 0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, -0.381)
    );

    public MecanumDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize each motor
        frontLeft = hardwareMap.get(DcMotorEx.class, IDs.FRONT_LEFT_ID);
        frontRight = hardwareMap.get(DcMotorEx.class, IDs.FRONT_RIGHT_ID);
        backLeft = hardwareMap.get(DcMotorEx.class, IDs.BACK_LEFT_ID);
        backRight = hardwareMap.get(DcMotorEx.class, IDs.BACK_RIGHT_ID);

        // Mecanum requires you to reverse two motors, depending on your convention
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Set brake mode
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Setting individual PIDF coefficients for each motor
        frontLeft.setVelocityPIDFCoefficients(
                PIDFCoefficients.kP, PIDFCoefficients.kI, PIDFCoefficients.kD, PIDFCoefficients.kFF
        );
        frontRight.setVelocityPIDFCoefficients(
                PIDFCoefficients.kP, PIDFCoefficients.kI, PIDFCoefficients.kD, PIDFCoefficients.kFF
        );
        backLeft.setVelocityPIDFCoefficients(
                PIDFCoefficients.kP, PIDFCoefficients.kI, PIDFCoefficients.kD, PIDFCoefficients.kFF
        );
        backRight.setVelocityPIDFCoefficients(
                PIDFCoefficients.kP, PIDFCoefficients.kI, PIDFCoefficients.kD, PIDFCoefficients.kFF
        );

        // Set the motors to use the encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);


        // Todo: check telemetry
        /*telemetry.addData("FrontLeft — mecanumDriveTrain", wheelSpeeds.frontLeftMetersPerSecond);
        telemetry.addData("FrontRight — mecanumDriveTrain", wheelSpeeds.frontRightMetersPerSecond);
        telemetry.addData("BackLeft — mecanumDriveTrain", wheelSpeeds.rearLeftMetersPerSecond);
        telemetry.addData("BackRight — mecanumDriveTrain", wheelSpeeds.rearRightMetersPerSecond);*/




        //setDesiredVelocities(wheelSpeeds);

        telemetry.addData("Estoy vivo", true);
        double conversionFactor = ConversionFactors.MOTOR_METERS_PER_SECOND_TO_DEGREES;
        frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
    }

    public void setDesiredVelocities(MecanumDriveWheelSpeeds wheelSpeeds) {
        telemetry.addData("Estoy vivo", true);
        double conversionFactor = ConversionFactors.MOTOR_METERS_PER_SECOND_TO_DEGREES;

        // Individual wheel speeds, after being multiplied by the conversion factor, is passed to the
        // setVelocity() method as degrees (value it takes, declared in the second parameter)
        /*frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond * conversionFactor, AngleUnit.DEGREES);*/
        frontLeft.setVelocity(80, AngleUnit.DEGREES);
    }

    public void stopMotors() {
        // TODO: check the setMotorDisable() method
        //frontLeft.setMotorDisable();

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}
