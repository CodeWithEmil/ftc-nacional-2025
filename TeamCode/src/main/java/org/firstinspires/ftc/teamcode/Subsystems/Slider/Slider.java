package org.firstinspires.ftc.teamcode.Subsystems.Slider;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Slider.SliderConstants.SliderIDs;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slider extends SubsystemBase {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;
    private final PIDController positionPIDController = new PIDController(0.008, 0.0, 0.0);
    double targetPosition = 0.0;

    public Slider(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor = hardwareMap.get(DcMotorEx.class, SliderIDs.sliderLeftMotorId);
        rightMotor = hardwareMap.get(DcMotorEx.class, SliderIDs.sliderRightMotorId);

        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        telemetry.addData("Hey! I'm the slider :)", true);
    }
}
