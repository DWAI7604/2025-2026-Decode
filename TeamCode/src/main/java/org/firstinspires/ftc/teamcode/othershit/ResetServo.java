package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Reset Servo", group="Linear OpMode")
public class ResetServo extends LinearOpMode {
    private Servo servo;

    @Override
    public void runOpMode () {
        servo = hardwareMap.get(Servo.class, "feeder");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(0);
            sleep(100);
        }
    }
}
