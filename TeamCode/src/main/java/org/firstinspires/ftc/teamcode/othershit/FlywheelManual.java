package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="flywheelManual", group="LinearOp")
public class FlywheelManual extends LinearOpMode {
    private DcMotor flywheel;
    private State state;

    public void runOpMode () {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        state = State.one;

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.0;

            switch (state) {
                case one:
                    power = -1.0;
                    break;
                case two:
                    power = -0.85;
                    break;
                case three:
                    power = -0.7;
                    break;
                case four:
                    power = -0.55;
                    break;
                case five:
                    power = -0.40;
                    break;
            }

            flywheel.setPower(power);

            if (gamepad1.a) {
                state = State.one;
            }
            if (gamepad1.b) {
                state = State.two;
            }
            if (gamepad1.y) {
                state = State.three;
            }
            if (gamepad1.x) {
                state = State.four;
            }
            if (gamepad1.dpad_right) {
                state = State.five;
            }
        }
    }

    public enum State {
        one, two, three, four, five
    }
}