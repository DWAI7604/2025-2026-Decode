package org.firstinspires.ftc.teamcode.turret;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PoseStorage;

@TeleOp(name="BoomBoomBoom", group="turret")
@Configurable
public class BoomBoomBoom extends LinearOpMode {

    private static double TICKS_PER_REV = 384.5;
    private static double TURRET_GEAR_RATIO = 24.0 / 136.0;

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    public static double AIM_X = 20;
    public static double AIM_Y = 20;

    public static double PID_P = 0;
    public static double PID_I = 0;
    public static double PID_D = 0;

    private PinpointLocalizer localizer;
    private Pose2d pose;

    private DcMotorEx turretMotor;

    private double robotHeading;
    private double motorHeading;
    private double turretHeading = 0;
    private double goalHeading;
    private double targetTurretHeading;
    private double errorHeading;

    private double power = 0;

    public void runOpMode() {

        localizer = new PinpointLocalizer(hardwareMap, 72.0 / 36653.0, PoseStorage.currentPose);
        pose = localizer.getPose();

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
//        flywheel.setDirection(DcMotor.Direction.REVERSE);
//        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(700, 3, 200, 0));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        waitForStart();

        while (opModeIsActive()) {

            localizer.update();
            pose = localizer.getPose();

            robotHeading = pose.heading.toDouble();
            motorHeading = turretMotor.getCurrentPosition() / TICKS_PER_REV;
            turretHeading = motorHeading * TURRET_GEAR_RATIO;
            goalHeading = Math.atan2(AIM_Y - pose.position.y, AIM_X - pose.position.x);
            targetTurretHeading = goalHeading - robotHeading;
            errorHeading = targetTurretHeading - turretHeading;

            power = PID_P * errorHeading;

            turretMotor.setPower(power);

            telemetry.addLine("see panels");

            panelsTelemetry.getTelemetry().addData("robotHeading", robotHeading);
            panelsTelemetry.getTelemetry().addData("goalHeading", goalHeading);
            panelsTelemetry.getTelemetry().addData("turretHeading", turretHeading);
            panelsTelemetry.getTelemetry().addData("targetTurretHeading", targetTurretHeading);
            panelsTelemetry.getTelemetry().addData("errorHeading", errorHeading);

            panelsTelemetry.getTelemetry().update();

        }

    }


}
