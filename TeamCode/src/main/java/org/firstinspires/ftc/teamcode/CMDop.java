package org.firstinspires.ftc.teamcode;


import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriver;

import java.util.function.Supplier;

@TeleOp(name = "CMD", group = "OpMode")
public class CMDop extends CommandOpMode {
    MecanumDriveTrain mecanumSubsystem = new MecanumDriveTrain(hardwareMap, telemetry);
    MecanumDriver mecanumDriver;

    @Override
    public void initialize() {
        mecanumDriver = new MecanumDriver(
            () -> (double) -gamepad1.left_stick_x,
            () -> (double) -gamepad1.left_stick_y,
            () -> (double) -gamepad1.right_stick_x,
            () -> (double) -gamepad1.right_stick_y
        );
    }

    public void configureBindings() {
        /*mecanumSubsystem.setDefaultCommand(CommandScheduler.() -> {
            mecanumDriver.apply(mecanumSubsystem);
        };
        new GamepadButton(gamepad1, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    otos.resetTracking();
                    imu.resetYaw();
                }));*/
    }
}