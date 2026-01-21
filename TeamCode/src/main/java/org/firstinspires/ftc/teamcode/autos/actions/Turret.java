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

    public static double AIM_X = -67;
    public static double AIM_Y = 72;

    public static double PID_P = 3;
//    public static double PID_I = 0;
//    public static double PID_D = 0;

    public static double LL_P = 0.015;

    private Limelight3A limelight;
    private LLResult result;
    public static boolean limelightActive = true;

    private DcMotorEx turretMotor;

    private double robotHeading;
    private double turretHeading = 0;
    private double goalHeading;
    private double targetTurretHeading;
    private double errorHeading;

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

            if (Math.abs( errorHeading ) > 20.3) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 5) {
                followGoalInitialized = false;
                turretMotor.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class Reset implements Action {
        private boolean resetInitialized = false;
        private int numOfCorrect;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!resetInitialized) {
                resetInitialized = true;
            }

            targetTurretHeading = 0;

            errorHeading = targetTurretHeading - turretHeading;

            double power = PID_P * Math.sqrt(Math.abs(errorHeading)) * Math.signum(errorHeading);

            if (power > POWER_CAP)
                power = POWER_CAP;
            else if (power < -POWER_CAP)
                power = -POWER_CAP;

            turretMotor.setPower(power);

            if (Math.abs( errorHeading ) > 20.3) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 5) {
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

    public Action reset() {
        return new Reset();
    }
}