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

    private static double blockerPosClosed = .11;
    private static double blockerPosOpened = .52;

    public static double launchTimerDelayToIntake_FAR = 0; //ms
    public static double launchTimerDelayToPassthrough_FAR = 0; //ms
    public static double launchTimerDelayToIntake_CLOSE = 100; //ms
    public static double launchTimerDelayToPassthrough_CLOSE = 0; //ms
    private static double launchTimerDelayToEnd = 1900; // ms

    private static double INTAKE_POWER = 1.0;
    public static double PASSTHROUGH_POWER_CLOSE = 0.7;
    public static double PASSTHROUGH_POWER_FAR = 0.32;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
//        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(600, 0.5, 30, 10));

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

//            packet.put("1.target flywheel", velocity);
//            packet.put("2.velocity flywheel", flywheel.getVelocity());
//            packet.put("3.error flywheel", Math.abs( flywheel.getVelocity() - velocity ));
////            packet.put("mode", "velocity");
//            packet.put("4.OUTPUT", Math.abs( flywheel.getVelocity() - velocity ) > 15);

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

    public class LaunchClose implements Action {
        private boolean launchCloseInitialized = false;
        private ElapsedTime launchCloseTimer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!launchCloseInitialized) {
                launchCloseInitialized = true;
                launchCloseTimer.reset();
            }

            blocker.setPosition(blockerPosOpened);

            if (launchCloseTimer.milliseconds() >= launchTimerDelayToPassthrough_CLOSE) {
                passthrough.setPower(PASSTHROUGH_POWER_CLOSE);
            }

            if (launchCloseTimer.milliseconds() >= launchTimerDelayToIntake_CLOSE) {
                intake.setPower(INTAKE_POWER);
            }

            if (launchCloseTimer.milliseconds() >= launchTimerDelayToEnd) {
                blocker.setPosition(blockerPosClosed);
                return false;
            }

            return true;
        }
    }

    public class LaunchFar implements Action {
        private boolean launchFarInitialized = false;
        private ElapsedTime launchFarTimer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!launchFarInitialized) {
                launchFarInitialized = true;
                launchFarTimer.reset();
            }

            blocker.setPosition(blockerPosOpened);

            if (launchFarTimer.milliseconds() >= launchTimerDelayToPassthrough_FAR) {
                passthrough.setPower(PASSTHROUGH_POWER_FAR);
            }

            if (launchFarTimer.milliseconds() >= launchTimerDelayToIntake_FAR) {
                intake.setPower(INTAKE_POWER);
            }

            if (launchFarTimer.milliseconds() >= launchTimerDelayToEnd) {
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


    public Action launchClose() {
        return new LaunchClose();
    }
    public Action launchFar() {
        return new LaunchFar();
    }

    public Action angle(double angle) {
        return new Angle(angle);
    }

    public Action velocity(double velocity) { return new Velocity(velocity); }

    public Action closeBlocker() { return new CloseBlocker(); }

    public Action stopAll() { return new StopAll(); }
}