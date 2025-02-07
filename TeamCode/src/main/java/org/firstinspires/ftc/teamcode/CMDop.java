package org.firstinspires.ftc.teamcode;


import androidx.core.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriver;

@TeleOp(name = "CMD-cutieputie", group = "OpMode")
public class CMDop extends CommandOpMode {
    GamepadEx controller;
    MecanumDriveTrain mecanumSubsystem;
    MecanumDriver mecanumDriver;

    @Override
    public void initialize() {
        mecanumSubsystem  = new MecanumDriveTrain(hardwareMap, telemetry);
        mecanumDriver = new MecanumDriver(
                () -> new Pair<>((double) -gamepad1.left_stick_x, (double) -gamepad1.left_stick_y),
                () -> new Pair<>((double) -gamepad1.right_stick_x, (double) -gamepad1.right_stick_y),
                hardwareMap,
                telemetry
        );

        // Initializing a new controller
        controller = new GamepadEx(gamepad1);

        // todo: Regresar este comando adentro de configureBindings() y hacer una prueba si todo funciona
        configureBindings();
    }

    public void configureBindings() {

        mecanumSubsystem.setDefaultCommand(
                new RunCommand(() -> mecanumDriver.apply(mecanumSubsystem), mecanumSubsystem));

        new GamepadButton(controller, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    mecanumDriver.getOTOS().resetTracking();
                    mecanumDriver.getIMU().resetYaw();
                }));
    }

    @Override
    public void runOpMode() {
        //telemetry.addData("RunOP", true);
        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (controller.getButton(GamepadKeys.Button.A)) {
                telemetry.addData("A button pressed", true);
            }
            mecanumSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));/*
            if (controller.getLeftY() > 0.0 || controller.getLeftX() > 0.0) {
                telemetry.addData("Current lecture of L stick Y", controller.getLeftY());
                telemetry.addData("Current lecture of L stick X", controller.getLeftX());
            }*/

            telemetry.update();
            run();
        }
        reset();
    }
}