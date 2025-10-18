package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Luca", group="LinearOp")
public class Luca extends LinearOpMode {
    private DcMotor flywheel;
    private Servo feeder;
    private State state;
    private FeedState feedState;
    private FeedState lastFeedState;

    public void runOpMode () {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feeder = hardwareMap.get(Servo.class, "feeder");
        state = State.one;
        feedState = FeedState.down;
        lastFeedState = FeedState.down;

        double power = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("servo position: ", feeder.getPosition());

//            if (feedState != lastFeedState) {
//                if (feedState == FeedState.down) {
//
//                } else if (feedState == FeedState.up) {
//
//                }
//                lastFeedState = feedState;
//            }

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

    public enum FeedState {
        up, down
    }
}