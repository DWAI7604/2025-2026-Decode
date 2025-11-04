package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="FlywheelTele", group="Linear OpMode")
public class FlywheelTele extends LinearOpMode {
    private DcMotorEx flywheel;
//    private FlywheelState flywheelState = FlywheelState.off;
//    private FlywheelState flywheelStatePrev = FlywheelState.off;

    private double flywheelTarVel = 0.0; // measured in RPM
    private double flywheelVel = 0.0; // measured in RPM
    private double flywheelPower = 0;

    private Servo feeder; // this is that lil flicky switch
    private FeederState feederState = FeederState.down;
    private ElapsedTime feederTimer = new ElapsedTime();

    private boolean useVel = true;

    private boolean b2Prev = false;
    private boolean a2Prev = false;
    private boolean x2Prev = false;
    private boolean y2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;
    private boolean bumperR2Prev = false;
    private boolean bumperL2Prev = false;

    @Override
    public void runOpMode () {
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(3, 0.5, 0.5, 0));

        feeder = hardwareMap.get(Servo.class, "feeder");
        feederTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            boolean updateFlywheel = false;
            if (gamepad2.b != b2Prev) {
                b2Prev = gamepad2.b;

                if (b2Prev) { // OFF
                    flywheelPower = 0.00;
                    flywheelTarVel = 0.0;
                    updateFlywheel = true;
                }
            } else if (gamepad2.a != a2Prev) {
                a2Prev = gamepad2.a;

                if (a2Prev) { // CLOSE
                    flywheelPower = 0.525;
                    flywheelTarVel = 1100;
//                    angleAdjust = 1.0;
//                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.x != x2Prev) {
                x2Prev = gamepad2.x;

                if (x2Prev) { // MEDIUM
                    flywheelPower = 0.60;
                    flywheelTarVel = 1280;
//                    angleAdjust = 0.6;
//                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.y != y2Prev) {
                y2Prev = gamepad2.y;

                if (y2Prev) { // FAR
                    flywheelPower = 0.725;
                    flywheelTarVel = 1540;
//                    angleAdjust = 0.15;
//                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            }

            if (gamepad2.dpad_up != dpadUp2Prev) {
                dpadUp2Prev = gamepad2.dpad_up;

                if (dpadUp2Prev) {
                    flywheelPower += 0.025;
                     flywheelTarVel += 60;
                    updateFlywheel = true;
                }
            } else if (gamepad2.dpad_down != dpadDown2Prev) {
                dpadDown2Prev = gamepad2.dpad_down;

                if (dpadDown2Prev) {
                    flywheelPower -= 0.025;
                    flywheelTarVel -= 60;
                    updateFlywheel = true;
                }
            }

            if (updateFlywheel) {
                if (!useVel) {
                    flywheelPower = Math.max(Math.min(flywheelPower, 0.85), 0); // caps flywheel from going above 85% or into reverse
                    flywheel.setPower(flywheelPower);
                } else {
                    flywheelTarVel = Math.max(Math.min(flywheelTarVel, 1800), 0);
                    flywheel.setVelocity(flywheelTarVel);
                }
            }

            flywheelVel = flywheel.getVelocity();


            if (gamepad2.left_bumper != bumperR2Prev) {
                bumperR2Prev = gamepad2.left_bumper;

                if (bumperR2Prev) {
                    useVel = !useVel;
                }
            }

            if (feederTimer.milliseconds() > 150) {

                if (feederState == FeederState.up) {
                    feeder.setPosition(.22);
                    feederState = FeederState.down;
                } else if (gamepad2.right_bumper != bumperR2Prev) {
                    bumperR2Prev = gamepad2.right_bumper;

                    if (bumperR2Prev) {
                        feeder.setPosition(.001);
                        feederState = FeederState.up;
                        feederTimer.reset();
                    }
                }
            }


            telemetry.addData("Power", flywheelPower);
            telemetry.addData("Velocity", flywheel.getVelocity());
            telemetry.addData("using vel", useVel);
            telemetry.addData("P val", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
            telemetry.addData("I val", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i);
            telemetry.addData("D val", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d);
            telemetry.addData("F val", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
//            telemetry.addData("Velocity (deg/s)", flywheel.getVelocity(AngleUnit.DEGREES));
//            telemetry.addData("Velocity (rpm)", flywheel.getVelocity(AngleUnit.DEGREES) / 360.0 * 60.0);
            telemetry.update();

        }
    }

    enum FlywheelState {
        off, close, middle, far
    }

    enum FeederState {
        up, down
    }
}
