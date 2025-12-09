package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private DcMotorEx flywheel;
    private DcMotor passthrough;
    private DcMotor intake;
    private Servo angleAdjustServo;
    private Servo blocker;

    private static double blockerPosClosed = .59;
    private static double blockerPosOpened = .78;

    private static double launchTimerDelayToIntake = 600; //ms
    private static double launchTimerDelayToPassthrough = 10; //ms
    private static double launchTimerDelayToEnd = 1900; // ms

    private static double INTAKE_POWER = 1.0;
    private static double PASSTHROUGH_POWER = 0.5;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(700, 3, 200, 0));

        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
        passthrough.setDirection(DcMotor.Direction.REVERSE);
        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");

        blocker = hardwareMap.get(Servo.class, "blocker");
    }

    public class Velocity implements Action {
        private boolean velocityInitialized = false;
        double velocity;
        int numOfCorrect = 0;

        public Velocity(double velocity) {
            this.velocity = velocity;
            velocityInitialized = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!velocityInitialized) {
                flywheel.setVelocity(velocity);
                velocityInitialized = true;
            }

            flywheel.setVelocity(velocity);

            packet.put("1.target flywheel", velocity);
            packet.put("2.velocity flywheel", flywheel.getVelocity());
            packet.put("3.error flywheel", Math.abs( flywheel.getVelocity() - velocity ));
//            packet.put("mode", "velocity");
            packet.put("4.OUTPUT", Math.abs( flywheel.getVelocity() - velocity ) > 15);

            if (Math.abs( flywheel.getVelocity() - velocity ) > 25) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 5) {
                velocityInitialized = false;
                return false;
            }
            return true;
        }
    }

    public class Launch implements Action {
        private boolean launchInitialized = false;
        private ElapsedTime launchTimer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!launchInitialized) {
                launchInitialized = true;
                launchTimer.reset();
            }

            blocker.setPosition(blockerPosOpened);

            if (launchTimer.milliseconds() >= launchTimerDelayToPassthrough) {
                passthrough.setPower(PASSTHROUGH_POWER);
            }

            if (launchTimer.milliseconds() >= launchTimerDelayToIntake) {
                intake.setPower(INTAKE_POWER);
            }

            if (launchTimer.milliseconds() >= launchTimerDelayToEnd) {
                blocker.setPosition(blockerPosClosed);
                return false;
            }

            return true;
        }
    }

    public class Angle implements Action {
        private boolean angleInitialized = false;
        private ElapsedTime angleTimer = new ElapsedTime();
        private double angle;

        public Angle(double angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!angleInitialized) {
                angleInitialized = true;
                angleTimer.reset();
                angleAdjustServo.setPosition(angle);
            }

            return angleTimer.milliseconds() < 50;
        }
    }

    public class CloseBlocker implements Action {
        private boolean blockerInitialized = false;
        private ElapsedTime blockerTimer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!blockerInitialized) {
                blockerInitialized = true;
                blockerTimer.reset();
                blocker.setPosition(blockerPosClosed);
            }

            return blockerTimer.milliseconds() < 10;
        }
    }

    public class StopAll implements Action {
        private boolean stopInitialized = false;
        private ElapsedTime stopTimer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!stopInitialized) {
                stopInitialized = true;
                stopTimer.reset();

                blocker.setPosition(blockerPosClosed);
                passthrough.setPower(0);
                intake.setPower(0);
                flywheel.setVelocity(0);
            }

            return stopTimer.milliseconds() < 10;
        }
    }


    public Action launch() {
        return new Launch();
    }

    public Action angle(double angle) {
        return new Angle(angle);
    }

    public Action velocity(double velocity) { return new Velocity(velocity); }

    public Action closeBlocker() { return new CloseBlocker(); }

    public Action stopAll() { return new StopAll(); }
}