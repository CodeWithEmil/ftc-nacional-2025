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
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);


        // Todo: check telemetry
        /*telemetry.addData("FrontLeft — mecanumDriveTrain", wheelSpeeds.frontLeftMetersPerSecond);
        telemetry.addData("FrontRight — mecanumDriveTrain", wheelSpeeds.frontRightMetersPerSecond);
        telemetry.addData("BackLeft — mecanumDriveTrain", wheelSpeeds.rearLeftMetersPerSecond);
        telemetry.addData("BackRight — mecanumDriveTrain", wheelSpeeds.rearRightMetersPerSecond);*/

        setDesiredVelocities(wheelSpeeds);
    }

    public void setDesiredVelocities(MecanumDriveWheelSpeeds wheelSpeeds) {
        telemetry.addData("Estoy vivo  MUEVANME", true);
        double conversionFactor = ConversionFactors.MOTOR_METERS_PER_SECOND_TO_DEGREES;

        telemetry.addData("FrontLeft — ConversionFactor", wheelSpeeds.frontLeftMetersPerSecond * conversionFactor);
        telemetry.addData("FrontRight — ConversionFactor", wheelSpeeds.frontRightMetersPerSecond * conversionFactor);
        telemetry.addData("BackLeft — ConversionFactor", wheelSpeeds.rearLeftMetersPerSecond * conversionFactor);
        telemetry.addData("BackRight — ConversionFactor", wheelSpeeds.rearRightMetersPerSecond * conversionFactor);

        // Individual wheel speeds, after being multiplied by the conversion factor, is passed to the
        // setVelocity() method as degrees (value it takes, declared in the second parameter)
        frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * conversionFactor, AngleUnit.DEGREES);
        backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond * conversionFactor, AngleUnit.DEGREES);

        //frontLeft.setPower(0.6);
        //frontRight.setPower(0.6);
        //backLeft.setPower(0.6);
        //frontLeft.setPower(wheelSpeeds.normalize(40));
        //frontLeft.setPower(wheelSpeeds.frontLeftMetersPerSecond * 0.3);
        //telemetry.addData("frontLeft — Power set", wheelSpeeds.frontLeftMetersPerSecond * 0.3);
        //telemetry.addData("frontLeftMPS", wheelSpeeds.frontLeftMetersPerSecond);

        //frontLeft.setVelocity(80, AngleUnit.DEGREES);
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
