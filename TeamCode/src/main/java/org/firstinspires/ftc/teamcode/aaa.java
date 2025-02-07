package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain.MecanumConstants;

@TeleOp(name = "CMD-aaa", group = "OpMode")
public class aaa extends CommandOpMode {

    DcMotorEx m;

    @Override
    public void initialize() {
        m = hardwareMap.get(DcMotorEx.class, MecanumConstants.IDs.FRONT_LEFT_ID);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void runOpMode() {
        //telemetry.addData("RunOP", true);
        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            m.setPower(1.0);


            telemetry.update();
            run();
        }
        reset();
    }
}