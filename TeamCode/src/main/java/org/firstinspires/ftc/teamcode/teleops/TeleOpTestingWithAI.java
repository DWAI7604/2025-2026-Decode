package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PoseStorage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name="TeleOp - Testing New", group="Linear OpMode")
@Configurable
public class TeleOpTestingWithAI extends LinearOpMode {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    // --- Configurable tuning parameters ---
    public static double MIN_SPEED_SCALER = 0.15;
    public static double SPEED_SCALER_RANGE = 0.8; // scaler = 1 - RANGE * trigger

    public static double JOYSTICK_DEADZONE = 0.05;
    public static boolean USE_SQUARED_INPUT = true;
    public static boolean USE_EXPONENTIAL_INPUT = false; // if true, use INPUT_EXPONENT instead of squared
    public static double INPUT_EXPONENT = 2.0; // exponent applied when USE_EXPONENTIAL_INPUT is true

    public static double AIM_P = 0.5; // proportional gain for auto-aim
    public static double AIM_DEADBAND = 0.02; // radians
    public static double AIM_MAX_YAW = 1.0; // clamp yaw output

    // goal coords and flywheel presets (expose for on-field tuning)
    public static double GOAL_X = -72 + 3;
    public static double GOAL_Y = 72 - 5;

    public static double FLYWHEEL_CLOSE_TPS = 1000; // ticks-per-second
    public static double FLYWHEEL_MEDIUM_TPS = 1400; // ticks-per-second
    public static double FLYWHEEL_FAR_TPS = 1580; // ticks-per-second

    // optional slew-rate limiter for smooth inputs
    public static boolean USE_SLEW_RATE = false;
    public static double SLEW_MAX_AXIS_DELTA = 3.0; // units per second for axial/lateral/yaw (unitless joystick units)
    

    // --- End config ---

    private PinpointLocalizer localizer;
    private Pose2d pose;

    private DcMotor leftFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor rightBackDriveMotor;

    private DcMotorEx flywheel;
    private DcMotor intake;
    private Servo feeder; // this is that lil flicky switch
    private Servo angleAdjustServo;
    private DcMotor passthrough;

    private boolean passthroughOn = false;
    private double flywheelTarVel = 0;
    private double angleAdjust;
    private IntakeState intakeState = IntakeState.off;
    private FeederState feederState = FeederState.down;
    private ElapsedTime feederTimer = new ElapsedTime();
    private DriveState driveState = DriveState.fieldCentric;

    // previous-button state trackers
    private boolean a1Prev = false;
    private boolean b1Prev = false;
    private boolean x1Prev = false;
    private boolean y1Prev = false;
    private boolean a2Prev = false;
    private boolean b2Prev = false;
    private boolean x2Prev = false;
    private boolean y2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;
    private boolean dpadLeft2Prev = false;
    private boolean dpadRight2Prev = false;
    private boolean bumperR2Prev = false;

    private ElapsedTime loopTimer = new ElapsedTime();

    // slew limiters (created in initHardware)
    private SlewRateLimiter axialLimiter;
    private SlewRateLimiter lateralLimiter;
    private SlewRateLimiter yawLimiter;
    

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        feederTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            localizer.update();
            pose = localizer.getPose();

            // ===== FLYWHEEL POWER & PRESETS =====
            boolean updateFlywheel = false;
            if (gamepad2.b != b2Prev) {
                b2Prev = gamepad2.b;

                if (b2Prev) { // OFF
                    flywheelTarVel = 0;
                    updateFlywheel = true;
                }
            } else if (gamepad2.a != a2Prev) {
                a2Prev = gamepad2.a;

                if (a2Prev) { // CLOSE
                    flywheelTarVel = FLYWHEEL_CLOSE_TPS;
                    angleAdjust = 1.0;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.x != x2Prev) {
                x2Prev = gamepad2.x;

                if (x2Prev) { // MEDIUM
                    flywheelTarVel = FLYWHEEL_MEDIUM_TPS;
                    angleAdjust = 0;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.y != y2Prev) {
                y2Prev = gamepad2.y;

                if (y2Prev) { // FAR
                    flywheelTarVel = FLYWHEEL_FAR_TPS;
                    angleAdjust = 0;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            }

            if (gamepad2.dpad_up != dpadUp2Prev) {
                dpadUp2Prev = gamepad2.dpad_up;

                if (dpadUp2Prev) {
                    flywheelTarVel += 10;
                    updateFlywheel = true;
                }
            } else if (gamepad2.dpad_down != dpadDown2Prev) {
                dpadDown2Prev = gamepad2.dpad_down;

                if (dpadDown2Prev) {
                    flywheelTarVel -= 10;
                    updateFlywheel = true;
                }
            }

            if (updateFlywheel) {
                flywheel.setVelocity(flywheelTarVel);
            }

            if (Math.abs(flywheel.getVelocity() - flywheelTarVel) < 50 && flywheelTarVel != 0) {
                gamepad2.rumble(0.5, 0.5, 50);
            }

            // ===== FEEDER FLICKING =====
            if (feederTimer.milliseconds() < 50) {
                passthrough.setPower(-0.5);
                passthroughOn = false;
            } else if (feederTimer.milliseconds() < 250) {
                feeder.setPosition(.001);
                feederState = FeederState.up;
                passthrough.setPower(0);
            }

            if (feederTimer.milliseconds() > 250) {
                if (feederState == FeederState.up) {
                    feeder.setPosition(0.4);
                    feederState = FeederState.down;
                } else if (gamepad2.right_bumper != bumperR2Prev) {
                    bumperR2Prev = gamepad2.right_bumper;

                    if (bumperR2Prev) {
                        feederTimer.reset();
                    }
                }
            }

            // ===== INTAKE STATE CHANGING =====
            if (a1Prev != gamepad1.a) {
                a1Prev = gamepad1.a;

                if (a1Prev) {
                    if (intakeState == IntakeState.on) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(1);
                        intakeState = IntakeState.on;
                    }
                }

            } else if (b1Prev != gamepad1.b) {
                b1Prev = gamepad1.b;

                if (b1Prev) {
                    if (intakeState == IntakeState.reject) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(-1);
                        intakeState = IntakeState.reject;
                    }
                }
            }

            // ===== PASSTHROUGH =====
            if (gamepad1.dpad_up) {
                passthrough.setPower(0.5);
                passthroughOn = true;
            } else if (gamepad1.dpad_right) {
                passthrough.setPower(0);
                passthroughOn = false;
            } else if (gamepad1.dpad_down) {
                passthrough.setPower(-0.5);
                passthroughOn = false;
            }

            if (passthroughOn) {
                gamepad1.rumble(0.5, 0.5, 50);
            }

            // ===== ADJUST ANGLE OF FLYWHEEL =====
            boolean updateAngleOfFlywheel = false;
            if (gamepad2.dpad_left != dpadLeft2Prev) {
                dpadLeft2Prev = gamepad2.dpad_left;

                if (dpadLeft2Prev) {
                    angleAdjust += 0.05;
                    updateAngleOfFlywheel = true;
                }
            } else if (gamepad2.dpad_right != dpadRight2Prev) {
                dpadRight2Prev = gamepad2.dpad_right;

                if (dpadRight2Prev) {
                    angleAdjust -= 0.05;
                    updateAngleOfFlywheel = true;
                }
            }
            if (updateAngleOfFlywheel) {
                angleAdjust = Math.max(Math.min(angleAdjust, 1.0), 0);
                angleAdjustServo.setPosition(angleAdjust);
            }

            // ===== RESET POSITION OF ROBOT =====
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                localizer.setPose(new Pose2d(72 - 9, -72 + 30, -Math.PI / 2.0));
            }

            // ===== DRIVE THE ROBOT =====
            if (x1Prev != gamepad1.x) {
                x1Prev = gamepad1.x;

                if (x1Prev) {
                    if (driveState == DriveState.fieldCentric) {
                        driveState = DriveState.robotCentric;
                    } else {
                        driveState = DriveState.fieldCentric;
                    }
                }

            } else if (y1Prev != gamepad1.y) {
                y1Prev = gamepad1.y;

                if (y1Prev) {
                    if (driveState == DriveState.autoAim) {
                        driveState = DriveState.fieldCentric;
                    } else {
                        driveState = DriveState.autoAim;
                    }
                }
            }

            mecanumDrive();

            telemetry.addData("heading", pose.heading.toDouble());
            telemetry.addData("angle of flywheel", angleAdjust);
            telemetry.addData("velocity of flywheel", flywheel.getVelocity());
            telemetry.addData("flywheel target", flywheelTarVel);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.update();

            panelsTelemetry.getTelemetry().addData("flywheel velocity", flywheel.getVelocity());
            panelsTelemetry.getTelemetry().addData("flywheel target", flywheelTarVel);
            panelsTelemetry.getTelemetry().update();

            // small loop pacing if needed
            // sleep(1);
        }
    }

    private void initHardware() {
        localizer = new PinpointLocalizer(hardwareMap, 72.0 / 36653.0, PoseStorage.currentPose);

        // Drive motors
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Other devices
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(800, 0.2, 0, 0));
        feeder = hardwareMap.get(Servo.class, "feeder");
        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
        passthrough.setDirection(DcMotor.Direction.REVERSE);
        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");

        // sensible defaults
        angleAdjust = 0.0;
        angleAdjustServo.setPosition(angleAdjust);
        flywheelTarVel = 0.0;
        feeder.setPosition(0.4);

        // create slew limiters (they internally track time) - created regardless so tests can toggle runtime
        axialLimiter = new SlewRateLimiter(SLEW_MAX_AXIS_DELTA);
        lateralLimiter = new SlewRateLimiter(SLEW_MAX_AXIS_DELTA);
        yawLimiter = new SlewRateLimiter(SLEW_MAX_AXIS_DELTA);
    // flywheel-specific slew limiter removed; flywheel setVelocity will use flywheelTarVel directly
    }

    private void mecanumDrive() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double heading = pose.heading.toDouble();
        heading = heading - Math.PI / 2.0; // adjusted for red's POV

        double yaw = 0;
        double axial = 0;
        double lateral = 0;

        // get raw stick inputs
        double rawLeftY = gamepad1.left_stick_y;
        double rawLeftX = gamepad1.left_stick_x;
        double rawRightX = gamepad1.right_stick_x;

        // apply deadzone
        axial = applyDeadzone(rawLeftY, JOYSTICK_DEADZONE);
        lateral = applyDeadzone(rawLeftX, JOYSTICK_DEADZONE);
        yaw = applyDeadzone(-rawRightX, JOYSTICK_DEADZONE); // keep same sign convention

        // optional squared input for finer control near center
        if (USE_EXPONENTIAL_INPUT) {
            axial = Math.signum(axial) * Math.pow(Math.abs(axial), INPUT_EXPONENT);
            lateral = Math.signum(lateral) * Math.pow(Math.abs(lateral), INPUT_EXPONENT);
            yaw = Math.signum(yaw) * Math.pow(Math.abs(yaw), INPUT_EXPONENT);
        } else if (USE_SQUARED_INPUT) {
            axial = squaredScale(axial);
            lateral = squaredScale(lateral);
            yaw = squaredScale(yaw);
        }

        if (driveState == DriveState.fieldCentric) {
            // rotate joystick vector by heading
            double tempAxial = (axial * Math.cos(heading) + lateral * Math.sin(heading));
            double tempLateral = (axial * Math.sin(heading) - lateral * Math.cos(heading));
            axial = tempAxial;
            lateral = tempLateral;
        } else if (driveState == DriveState.robotCentric) {
            // axial/lateral already set
            lateral = -lateral; // keep same sign as original robot-centric implementation
        } else if (driveState == DriveState.autoAim) {
            // compute yaw from auto-aim controller
            yaw = computeAutoAimYaw();
            // still keep field-centric translation so drivers can move while aiming
            double tempAxial = (axial * Math.cos(heading) + lateral * Math.sin(heading));
            double tempLateral = (axial * Math.sin(heading) - lateral * Math.cos(heading));
            axial = tempAxial;
            lateral = tempLateral;
        }

        // apply optional slew-rate limiting to the final translation/rotation commands
        if (USE_SLEW_RATE && axialLimiter != null) {
            axial = axialLimiter.apply(axial);
            lateral = lateralLimiter.apply(lateral);
            yaw = yawLimiter.apply(yaw);
        }

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

        leftFrontPower = (axial + lateral + yaw) / denominator;
        leftBackPower = (axial - lateral + yaw) / denominator;
        rightFrontPower = (axial - lateral - yaw) / denominator;
        rightBackPower = (axial + lateral - yaw) / denominator;

        // apply speed scaler (left trigger: more press -> slower)
        double speedScaler = 1.0 - SPEED_SCALER_RANGE * gamepad1.left_trigger; // full trigger -> ~1 - RANGE
        speedScaler = Math.max(MIN_SPEED_SCALER, Math.min(speedScaler, 1.0));

        leftFrontPower *= speedScaler;
        leftBackPower *= speedScaler;
        rightFrontPower *= speedScaler;
        rightBackPower *= speedScaler;

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);

        telemetry.addData("heading", heading);
        telemetry.addData("speedScaler", speedScaler);
    }

    private double computeAutoAimYaw() {
        // compute desired yaw to turn robot toward goal using atan2 + angle wrap + P controller
        double robotHeading = pose.heading.toDouble();

    double xDist = GOAL_X - pose.position.x;
    double yDist = GOAL_Y - pose.position.y;

        double goalHeading = Math.atan2(yDist, xDist);
        double error = wrapAngleRad(goalHeading - robotHeading); // shortest signed angle

        double yaw = AIM_P * error; // P controller

        // deadband
        if (Math.abs(error) < AIM_DEADBAND) yaw = 0;

        // clamp
        yaw = Math.max(-AIM_MAX_YAW, Math.min(AIM_MAX_YAW, yaw));

        telemetry.addData("goalHeading", goalHeading);
        telemetry.addData("aimError", error);

        return yaw;
    }

    private double wrapAngleRad(double ang) {
        // normalize to [-PI, PI)
        ang = (ang + Math.PI) % (2.0 * Math.PI);
        if (ang < 0) ang += 2.0 * Math.PI;
        return ang - Math.PI;
    }

    private double applyDeadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }

    private double squaredScale(double v) {
        return Math.signum(v) * v * v;
    }

    // Simple slew-rate limiter: limits the rate of change of the output value
    private static class SlewRateLimiter {
        private final double maxDeltaPerSecond;
        private double last = 0.0;
        private long lastTsNs = System.nanoTime();

        SlewRateLimiter(double maxDeltaPerSecond) {
            this.maxDeltaPerSecond = Math.max(maxDeltaPerSecond, 1e-9);
        }

        synchronized double apply(double input) {
            long now = System.nanoTime();
            double dt = (now - lastTsNs) / 1e9;
            if (dt <= 0) dt = 1e-9;
            double maxDelta = maxDeltaPerSecond * dt;
            double delta = input - last;
            if (delta > maxDelta) delta = maxDelta;
            if (delta < -maxDelta) delta = -maxDelta;
            last += delta;
            lastTsNs = now;
            return last;
        }
    }

    enum DriveState {
        fieldCentric, autoAim, robotCentric
    }

    enum IntakeState {
        off, on, reject
    }

    enum FeederState {
        up, down
    }
}
