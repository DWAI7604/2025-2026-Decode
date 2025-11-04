package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="TestingFeeder", group="Robot")
public class TestingFeeder extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo feeder = hardwareMap.get(Servo.class, "feeder");

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

//        while (opModeIsActive()) {
//            double position = 0.1;
//
//            position = 0.10 * Math.sin(timer.seconds()) + 0.10;
//
//            feeder.setPosition(position);
//
//            telemetry.addData("target: ", position);
//            telemetry.addData("actual: ", feeder.getPosition());
//            telemetry.update();
//        }

        feeder.setPosition(0.001);
//        while (timer.milliseconds() < 3000) {
//            telemetry.addData("target: ", "0.001");
//            telemetry.addData("actual: ", feeder.getPosition());
//            telemetry.update();
////        }
//        timer.reset();
        sleep(1000);
//        telemetry.addData("target: ", "0.001");
//        telemetry.addData("actual: ", feeder.getPosition());
//        telemetry.update();
//        sleep(1000);
//        telemetry.addData("target: ", "0.001");
//        telemetry.addData("actual: ", feeder.getPosition());
//        telemetry.update();
//        sleep(1000);

        feeder.setPosition(0.22);
//        while (timer.milliseconds() < 3000) {
//            telemetry.addData("target: ", "0.22");
//            telemetry.addData("actual: ", feeder.getPosition());
//            telemetry.update();
//        }
//        timer.reset();
        sleep(1000);
//        telemetry.addData("target: ", "0.22");
//        telemetry.addData("actual: ", feeder.getPosition());
//        telemetry.update();
//        sleep(1000);
//        telemetry.addData("target: ", "0.22");
//        telemetry.addData("actual: ", feeder.getPosition());
//        telemetry.update();
//        sleep(1000);

        feeder.setPosition(0.001);
//        while (timer.milliseconds() < 3000) {
//            telemetry.addData("target: ", "0.001");
//            telemetry.addData("actual: ", feeder.getPosition());
//            telemetry.update();
////        }
//
//        sleep(1000);
//        telemetry.addData("target: ", "0.001");
//        telemetry.addData("actual: ", feeder.getPosition());
//        telemetry.update();
//        sleep(1000);
//        telemetry.addData("target: ", "0.001");
//        telemetry.addData("actual: ", feeder.getPosition());
//        telemetry.update();
        sleep(1000);

    }
}
