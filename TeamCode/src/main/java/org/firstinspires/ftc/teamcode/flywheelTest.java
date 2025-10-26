package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class flywheelTest extends RobotLinearOpMode {

    private DcMotor flywheel;
    private Servo feeder;
    private double power;
    private boolean aIsPressed;
    private boolean bIsPressed;

    public void runOpMode() {

       flywheel = hardwareMap.get(DcMotor.class, "flywheel");
       feeder = hardwareMap.get(Servo.class, "feeder");

       power = 0;




        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                power = .65;
            } else if (gamepad1.b) {
                power = 0;
            } else if (gamepad1.x) {
                power = .7;
            } else if (gamepad1.y) {
                power = .75;
            } else if (gamepad1.dpad_up) {
                power = .6;
            } else if (gamepad1.dpad_left) {
                power = .55;
            } else if (gamepad1.dpad_right) {
                power = .5;
            } else if (gamepad1.dpad_down) {
                power = .45;
            }

            if (gamepad1.right_bumper) {

                feeder.setPosition(.55);
            } else if (gamepad1.left_bumper) {

                feeder.setPosition(.77);
            }

            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setPower(power);

            telemetry.addData("Power", power);
            telemetry.update();

        }

    }
}
