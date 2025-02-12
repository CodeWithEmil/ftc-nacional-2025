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

import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumDriver;

@TeleOp(name = "CMD-cutieputie", group = "OpMode")
public class CMDop extends CommandOpMode {
    GamepadEx controller;
    MecanumDriveTrain mecanumSubsystem;
    MecanumDriver mecanumDriver;

    // Arm subsystem
    Arm arm;

    @Override
    public void initialize() {
        mecanumSubsystem  = new MecanumDriveTrain(hardwareMap, telemetry);
        mecanumDriver = new MecanumDriver(
                () -> new Pair<>((double) -gamepad1.left_stick_x, (double) -gamepad1.left_stick_y),
                () -> new Pair<>((double) gamepad1.right_stick_x, (double) -gamepad1.right_stick_y),
                hardwareMap,
                telemetry
        );

        // Initializing a new controller
        controller = new GamepadEx(gamepad1);
        mecanumDriver.otos.resetTracking();
        mecanumDriver.imu.resetYaw();

        // Arm declaration
        arm = new Arm(hardwareMap, telemetry);

        configureBindings();
    }

    public void configureBindings() {

        mecanumSubsystem.setDefaultCommand(new RunCommand(() -> mecanumDriver.apply(mecanumSubsystem), mecanumSubsystem));

        new GamepadButton(controller, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    mecanumDriver.getOTOS().resetTracking();
                    mecanumDriver.getIMU().resetYaw();
                }));

        new GamepadButton(controller, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    arm.changePosition();
                }));
    }

    /*@Override
    public void run() {
        if (controller.getButton(GamepadKeys.Button.A)) {
            telemetry.addData("A button pressed", true);
        }

        telemetry.update();
        super.run();
    }*/

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (controller.getButton(GamepadKeys.Button.A)) {
                arm.changePosition();
            }
            //mecanumSubsystem.drive(new ChassisSpeeds(0.5, 0.5, 0.5));
            /*if (controller.getLeftY() > 0.0 || controller.getLeftX() > 0.0) {
                telemetry.addData("Current lecture of L stick Y", controller.getLeftY());
                telemetry.addData("Current lecture of L stick X", controller.getLeftX());
            }

            telemetry.update();
            // 8-feb-2025 acabo de leer en la documentación que llamar run() es malo
            // 8-feb-2025 porque te rompe el ciclo de comandos que se inicializan cuando llamas
            // 8-feb-2025 a waitForStart(), entonces comenté run() para que no interfiera*/
            telemetry.update();
            run();
        }
        reset();

    }
}