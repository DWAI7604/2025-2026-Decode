package org.firstinspires.ftc.teamcode.othershit;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
//@Disabled

@Configurable
@TeleOp(name="angleAdjust = 0.0")
public class ResetAngleAdjust extends LinearOpMode {
    private Servo servo;


    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    public static double position = 0.0;

    @Override
    public void runOpMode () {
        servo = hardwareMap.get(Servo.class, "angleAdjust");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(position);
            sleep(100);
        }
    }
}
