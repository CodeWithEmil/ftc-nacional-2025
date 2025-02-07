package org.firstinspires.ftc.teamcode;


import androidx.core.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriver;

import java.util.function.Supplier;

@TeleOp(name = "CMD", group = "OpMode")
public class CMDop extends CommandOpMode {
    GamepadEx controller = new GamepadEx(gamepad1);
    MecanumDriveTrain mecanumSubsystem = new MecanumDriveTrain(hardwareMap, telemetry);
    MecanumDriver mecanumDriver;

    @Override
    public void initialize() {
        mecanumDriver = new MecanumDriver(
                () -> new Pair<>(-controller.getLeftX(), -controller.getLeftY()),
                () -> new Pair<>(-controller.getRightX(), -controller.getRightY())
        );
    }

    public void configureBindings() {
        mecanumSubsystem.setDefaultCommand(
                new RunCommand(() -> mecanumDriver.apply(mecanumSubsystem), mecanumSubsystem));

        new GamepadButton(controller, GamepadKeys.Button.START)
                .toggleWhenActive(new InstantCommand(() -> {
                    mecanumDriver.getOTOS().resetTracking();
                    mecanumDriver.getIMU().resetYaw();
                }));
    }
}