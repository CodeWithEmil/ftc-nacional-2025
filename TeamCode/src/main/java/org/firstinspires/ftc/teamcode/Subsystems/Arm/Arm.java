package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmConstants;

public class Arm extends SubsystemBase {
    private Telemetry telemetry;
    private final Servo rightServo;
    private final Servo leftServo;
    private double iteration = 0;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        rightServo = hardwareMap.get(Servo.class, ArmConstants.Ids.armRightServo);
        leftServo = hardwareMap.get(Servo.class, ArmConstants.Ids.armLeftServo);

        leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void goToPosition(double position) {
        rightServo.setPosition(position / 180.0);
        leftServo.setPosition(position / 180.0);
    }

    public void changePosition() {
        if (this.iteration == 0) {
            goToPosition(ArmConstants.Positions.intakePosition);
            iteration = 1;
        } else if (this.iteration == 1) {
            goToPosition(ArmConstants.Positions.homePosition);
            iteration = 0;
        }

    }

}
