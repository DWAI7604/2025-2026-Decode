package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp
public class limitSwitchTesting extends LinearOpMode {

    TouchSensor limitSwitch;
    boolean limitSwitchWasPressed;
    double x = 0;
    @Override
    public void runOpMode() {
        limitSwitch = hardwareMap.get(TouchSensor.class, "limit");

        waitForStart();
        while(opModeIsActive()) {
            if (!limitSwitch.isPressed() && !limitSwitchWasPressed) {
                x++;
                limitSwitchWasPressed = true;
            } else if (limitSwitch.isPressed() && limitSwitchWasPressed) {
                limitSwitchWasPressed = false;
            }
            telemetry.addData("x:",x);
            telemetry.update();
        }
    }
}
