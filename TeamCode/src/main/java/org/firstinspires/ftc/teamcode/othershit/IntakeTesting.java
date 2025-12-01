package org.firstinspires.ftc.teamcode.othershit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeTesting extends LinearOpMode {

    private DcMotor intake;


    @Override
    public void runOpMode()  {

        intake = hardwareMap.get(DcMotor.class,"intake");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.setPower(.5);
            } else if (gamepad1.b) {
                intake.setPower(-.5);
            } else if (gamepad1.x) {
                intake.setPower(1);
            } else if (gamepad1.y) {
                intake.setPower(-1);
            }
         }

    }
}
