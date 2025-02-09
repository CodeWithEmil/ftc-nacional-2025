package org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants.IDs;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants.ConversionFactors;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants.PIDFCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;


import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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


        // 8-feb-2025 Telemetría añadida para ver si el encoder funciona
        // 8-feb-2025 Según la documentación, debería regresar RUN_USING_ENCODER
        telemetry.addData("Front Left Mode", frontLeft.getMode());
        telemetry.addData("Front Right Mode", frontRight.getMode());
        telemetry.addData("Back Left Mode", backLeft.getMode());
        telemetry.addData("Back Right Mode", backRight.getMode());
        telemetry.update();

    }

    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        setDesiredVelocities(wheelSpeeds);
    }

    public void setDesiredVelocities(MecanumDriveWheelSpeeds wheelSpeeds) {

        // 9-Feb-2025
        // Conversion factor seteado de metros por segundo (velocidad lineal) a
        // radianes por segundo (velocidad angular)
        double conversionFactor = ConversionFactors.MOTOR_MPS_TO_RADPS;

        double frontLeftVelocity = wheelSpeeds.frontLeftMetersPerSecond * conversionFactor;
        double frontRightVelocity = wheelSpeeds.frontRightMetersPerSecond * conversionFactor;
        double backLeftVelocity = wheelSpeeds.rearLeftMetersPerSecond * conversionFactor;
        double backRightVelocity = wheelSpeeds.rearRightMetersPerSecond * conversionFactor;

        // 8-feb-2025 Telemetría añadida para ver los valores pasados:
        // 8-feb-2025 Si son raros, entonces es tema del conversion factor
        telemetry.addData("frontLeft Velocity (rads/s)", frontLeftVelocity);
        telemetry.addData("frontRight Velocity (rads/s)", frontRightVelocity);
        telemetry.addData("backLeft Velocity (rads/s)", backLeftVelocity);
        telemetry.addData("backRight Velocity (rads/s)", backRightVelocity);
        telemetry.update();

        // Individual wheel speeds, after being multiplied by the conversion factor, is passed to the
        // setVelocity() method as radians (value it takes, declared in the second parameter)
        frontLeft.setVelocity(frontLeftVelocity, AngleUnit.RADIANS);
        frontRight.setVelocity(frontRightVelocity, AngleUnit.RADIANS);
        backLeft.setVelocity(backLeftVelocity, AngleUnit.RADIANS);
        backRight.setVelocity(backRightVelocity, AngleUnit.RADIANS);

        // 8-feb-2025 si esta telemetría jala, entonces el problema tiene que ver
        // 8-feb-2025 con que .setVelocity no se está llamando correctamente
        telemetry.addData("la velocidad fue seteada correctamente", true);
        telemetry.update();

        // 9-Feb-2025 !!! Update: si se llega a probar, utilizar AngleUnit.RADIANS
        // 8-feb-2025 También debemos probar el siguiente código:
        // 8-feb-2025 Si sigue sin moverse, es tema de los PIDs
        /*frontLeft.setVelocity(300, AngleUnit.DEGREES);
        frontRight.setVelocity(300, AngleUnit.DEGREES);
        backLeft.setVelocity(300, AngleUnit.DEGREES);
        backRight.setVelocity(300, AngleUnit.DEGREES);
        telemetry.addData("velocity fue seteado manualmente a # grados: ", 300);
        telemetry.update();*/
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
