package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private Pose2d pose;

    private DcMotorEx turret;

    private static double TICKS_PER_REV = 384.5;
    private static double TURRET_GEAR_RATIO = 24.0 / 136.0;
    public static double FUDGE = 6.2824155667973;

    public static double MAX_HEADING = Math.toRadians(100.0);
    public static double MIN_HEADING = Math.toRadians(-100.0);

    public static double POWER_CAP = 1;

//    public static double AIM_X = -70;
//    public static double AIM_Y = 69.5;

    public static double AIM_X = -63;
    public static double AIM_Y = 63;

    public static double AIM_X_FAR = -63;
    public static double AIM_Y_FAR = 66;

//    public static double AIM_X_CLOSE = -68;
//    public static double AIM_Y_CLOSE = 69.5;
//
//    public static double AIM_X_FAR = -67;
//    public static double AIM_Y_FAR = 70.5;

    public static double PID_P = 1;
//    public static double PID_I = 0;
//    public static double PID_D = 0;

    public static double LL_P = 0.015;

    private Limelight3A limelight;
    private LLResult result;
    public static boolean limelightActive = true;

    private DcMotorEx turretMotor;

//    private double robotHeading;
//    private double turretHeading = 0;
//    private double goalHeading;
//    private double targetTurretHeading;
//    private double errorHeading;

    public Turret(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // locate ID 24 (red goal)
        limelight.setPollRateHz(100);
//        limelight.start();
        limelight.pipelineSwitch(1); // locate ID 24 (red goal)

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
//        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(700, 3, 200, 0));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

//        this.pose = pose;
    }

    public class FollowGoalRed implements Action {
        private boolean followGoalInitialized = false;
        private double robotHeading, xPos, yPos;
        private int numOfCorrect;
        private ElapsedTime followGoalTimer = new ElapsedTime();

        private double turretHeading;
        private double goalHeading;
        private double targetTurretHeading;
        private double errorHeading;

        public FollowGoalRed(double robotHeading, double xPos, double yPos) {
            this.robotHeading = wrapHeading(robotHeading);
            this.xPos = xPos;
            this.yPos = yPos;
            followGoalInitialized = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!followGoalInitialized) {
                followGoalInitialized = true;
                followGoalTimer.reset();
            }

//            targetTurretHeading = robotHeading;

            turretHeading = wrapHeading(turretMotor.getCurrentPosition() / TICKS_PER_REV * (FUDGE) * TURRET_GEAR_RATIO);
            goalHeading = Math.atan2(AIM_Y - yPos, AIM_X - xPos);
            targetTurretHeading = wrapHeading(goalHeading - robotHeading + Math.toRadians(180));

            if (targetTurretHeading > MAX_HEADING)
                targetTurretHeading = MAX_HEADING;
            else if (targetTurretHeading < MIN_HEADING)
                targetTurretHeading = MIN_HEADING;

            errorHeading = targetTurretHeading - turretHeading;

            double power = PID_P * Math.sqrt(Math.abs(errorHeading)) * Math.signum(errorHeading);

            if (power > POWER_CAP)
                power = POWER_CAP;
            else if (power < -POWER_CAP)
                power = -POWER_CAP;

            turretMotor.setPower(power);

            if (Math.abs( errorHeading ) > 0.3) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 70) {
                followGoalInitialized = false;
                turretMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class FollowGoalRedFar implements Action {
        private boolean followGoalInitialized = false;
        private double robotHeading, xPos, yPos;
        private int numOfCorrect;
        private ElapsedTime followGoalTimer = new ElapsedTime();

        private double turretHeading;
        private double goalHeading;
        private double targetTurretHeading;
        private double errorHeading;

        public FollowGoalRedFar(double robotHeading, double xPos, double yPos) {
            this.robotHeading = wrapHeading(robotHeading);
            this.xPos = xPos;
            this.yPos = yPos;
            followGoalInitialized = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!followGoalInitialized) {
                followGoalInitialized = true;
                followGoalTimer.reset();
            }

//            targetTurretHeading = robotHeading;

            turretHeading = wrapHeading(turretMotor.getCurrentPosition() / TICKS_PER_REV * (FUDGE) * TURRET_GEAR_RATIO);
            goalHeading = Math.atan2(AIM_Y_FAR - yPos, AIM_X_FAR - xPos);
            targetTurretHeading = wrapHeading(goalHeading - robotHeading + Math.toRadians(180));

            if (targetTurretHeading > MAX_HEADING)
                targetTurretHeading = MAX_HEADING;
            else if (targetTurretHeading < MIN_HEADING)
                targetTurretHeading = MIN_HEADING;

            errorHeading = targetTurretHeading - turretHeading;

            double power = PID_P * Math.sqrt(Math.abs(errorHeading)) * Math.signum(errorHeading);

            if (power > POWER_CAP)
                power = POWER_CAP;
            else if (power < -POWER_CAP)
                power = -POWER_CAP;

            turretMotor.setPower(power);

            if (Math.abs( errorHeading ) > 0.3) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 70) {
                followGoalInitialized = false;
                turretMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class FollowGoalBlue implements Action {
        private boolean followGoalInitializedB = false;
        private double robotHeadingB, xPosB, yPosB;
        private int numOfCorrectB;
        private ElapsedTime followGoalTimerB = new ElapsedTime();

        private double turretHeadingB;
        private double goalHeadingB;
        private double targetTurretHeadingB;
        private double errorHeadingB;

        public FollowGoalBlue(double robotHeading, double xPos, double yPos) {
            this.robotHeadingB = wrapHeading(robotHeading);
            this.xPosB = xPos;
            this.yPosB = yPos;
            followGoalInitializedB = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!followGoalInitializedB) {
                followGoalInitializedB = true;
                followGoalTimerB.reset();
            }

//            targetTurretHeadingB = robotHeadingB;

            turretHeadingB = wrapHeading(turretMotor.getCurrentPosition() / TICKS_PER_REV * (FUDGE) * TURRET_GEAR_RATIO);
            goalHeadingB = Math.atan2(-AIM_Y - yPosB, AIM_X - xPosB);
            targetTurretHeadingB = wrapHeading(goalHeadingB - robotHeadingB + Math.toRadians(180));

            if (targetTurretHeadingB > MAX_HEADING)
                targetTurretHeadingB = MAX_HEADING;
            else if (targetTurretHeadingB < MIN_HEADING)
                targetTurretHeadingB = MIN_HEADING;

            errorHeadingB = targetTurretHeadingB - turretHeadingB;

            double power = PID_P * Math.sqrt(Math.abs(errorHeadingB)) * Math.signum(errorHeadingB);

            if (power > POWER_CAP)
                power = POWER_CAP;
            else if (power < -POWER_CAP)
                power = -POWER_CAP;

            turretMotor.setPower(power);

            if (Math.abs( errorHeadingB ) > 0.3) {
                numOfCorrectB = 0;
            } else {
                numOfCorrectB++;
            }

            packet.put("robot heading", robotHeadingB);
            packet.put("turret heading", turretHeadingB);
            packet.put("goal heading", goalHeadingB);
            packet.put("error heading", errorHeadingB);

            if (numOfCorrectB >= 70) {
                followGoalInitializedB = false;
                turretMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class FollowGoalBlueFar implements Action {
        private boolean followGoalInitializedB = false;
        private double robotHeadingB, xPosB, yPosB;
        private int numOfCorrectB;
        private ElapsedTime followGoalTimerB = new ElapsedTime();

        private double turretHeadingB;
        private double goalHeadingB;
        private double targetTurretHeadingB;
        private double errorHeadingB;

        public FollowGoalBlueFar(double robotHeading, double xPos, double yPos) {
            this.robotHeadingB = wrapHeading(robotHeading);
            this.xPosB = xPos;
            this.yPosB = yPos;
            followGoalInitializedB = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!followGoalInitializedB) {
                followGoalInitializedB = true;
                followGoalTimerB.reset();
            }

//            targetTurretHeadingB = robotHeadingB;

            turretHeadingB = wrapHeading(turretMotor.getCurrentPosition() / TICKS_PER_REV * (FUDGE) * TURRET_GEAR_RATIO);
            goalHeadingB = Math.atan2(-AIM_Y_FAR - yPosB, AIM_X_FAR - xPosB);
            targetTurretHeadingB = wrapHeading(goalHeadingB - robotHeadingB + Math.toRadians(180));

            if (targetTurretHeadingB > MAX_HEADING)
                targetTurretHeadingB = MAX_HEADING;
            else if (targetTurretHeadingB < MIN_HEADING)
                targetTurretHeadingB = MIN_HEADING;

            errorHeadingB = targetTurretHeadingB - turretHeadingB;

            double power = PID_P * Math.sqrt(Math.abs(errorHeadingB)) * Math.signum(errorHeadingB);

            if (power > POWER_CAP)
                power = POWER_CAP;
            else if (power < -POWER_CAP)
                power = -POWER_CAP;

            turretMotor.setPower(power);

            if (Math.abs( errorHeadingB ) > 0.3) {
                numOfCorrectB = 0;
            } else {
                numOfCorrectB++;
            }

            packet.put("robot heading", robotHeadingB);
            packet.put("turret heading", turretHeadingB);
            packet.put("goal heading", goalHeadingB);
            packet.put("error heading", errorHeadingB);

            if (numOfCorrectB >= 70) {
                followGoalInitializedB = false;
                turretMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class Reset implements Action {
        private boolean resetInitializedR = false;
        private int numOfCorrectR;

        private double turretHeadingR;
        private double goalHeadingR;
        private double targetTurretHeadingR;
        private double errorHeadingR;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!resetInitializedR) {
                resetInitializedR = true;
            }

            targetTurretHeadingR = 0;
            turretHeadingR = wrapHeading(turretMotor.getCurrentPosition() / TICKS_PER_REV * (FUDGE) * TURRET_GEAR_RATIO);

            errorHeadingR = targetTurretHeadingR - turretHeadingR;

            double power = PID_P * Math.sqrt(Math.abs(errorHeadingR)) * Math.signum(errorHeadingR);

            if (power > POWER_CAP)
                power = POWER_CAP;
            else if (power < -POWER_CAP)
                power = -POWER_CAP;

            turretMotor.setPower(power);

            if (Math.abs( errorHeadingR ) > 0.3) {
                numOfCorrectR = 0;
            } else {
                numOfCorrectR++;
            }

            if (numOfCorrectR >= 70) {
                turretMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public double wrapHeading (double heading) {
        return (heading + Math.PI) % (2*Math.PI) - Math.PI;
    }

    public Action followGoalRed(double xPos, double yPos, double robotHeading) {
        return new FollowGoalRed(robotHeading, xPos, yPos);
    }

    public Action followGoalRedFar(double xPos, double yPos, double robotHeading) {
        return new FollowGoalRedFar(robotHeading, xPos, yPos);
    }

    public Action followGoalBlue(double xPos, double yPos, double robotHeading) {
        return new FollowGoalBlue(robotHeading, xPos, yPos);
    }

    public Action followGoalBlueFar(double xPos, double yPos, double robotHeading) {
        return new FollowGoalBlueFar(robotHeading, xPos, yPos);
    }

    public Action reset() {
        return new Reset();
    }
}