package org.firstinspires.ftc.teamcode.othershit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
//@Configurable
@TeleOp(name="blocker = 0.5")
public class ResetBlocker extends LinearOpMode {
    private Servo servo;


//    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    public static double position = 0.5;

    @Override
    public void runOpMode () {
        servo = hardwareMap.get(Servo.class, "blocker");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(position);
            sleep(100);
        }
    }
}
