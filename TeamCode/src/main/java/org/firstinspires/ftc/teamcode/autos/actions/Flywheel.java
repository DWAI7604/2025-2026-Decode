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

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
//    private DcMotor passthrough;
    private DcMotor intake;
    private Servo angleAdjustServo;
    private Servo blocker;

    private static double blockerPosClosed = .34;
    private static double blockerPosOpened = .14;

    public static double launchTimerDelayToIntake_FAR = 0; //ms
//    public static double launchTimerDelayToPassthrough_FAR = 0; //ms
    public static double launchTimerDelayToIntake_CLOSE = 0; //ms
//    public static double launchTimerDelayToPassthrough_CLOSE = 0; //ms
    private static double launchTimerDelayToEnd = 1900; // ms

    private static double INTAKE_POWER_CLOSE = 0.825;
    private static double INTAKE_POWER_FAR = 0.775;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel2");
//        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(380, 0.2, 0, 0));
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(380, 0.2, 0, 0));

//        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
//        passthrough.setDirection(DcMotor.Direction.REVERSE);
//        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");

        blocker = hardwareMap.get(Servo.class, "blocker");
    }

    public class Velocity implements Action {
        private boolean velocityInitialized = false;
        private ElapsedTime velocityTimer = new ElapsedTime();
        double flywheel_step_diff = 50;
        double flywheel_step_pause = 30;
        private ElapsedTime lowerTimer = new ElapsedTime();
        double tarvel = 0;
        double tempvel = 0;
        int numOfCorrect = 0;

        public Velocity(double tarvel) {
            this.tarvel = tarvel;
            velocityInitialized = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!velocityInitialized) {
                velocityInitialized = true;
                velocityTimer.reset();
                lowerTimer.reset();
                tempvel = flywheel1.getVelocity();
            }

            if (tempvel != tarvel && lowerTimer.milliseconds() > flywheel_step_pause) {

                double error = tarvel - tempvel;

                if (Math.abs(error) < flywheel_step_diff)
                    tempvel = tarvel;

                else if (error > 0)
                    tempvel += flywheel_step_diff;

                else if (error < 0)
                    tempvel -= flywheel_step_diff;

                lowerTimer.reset();

            }

            flywheel1.setVelocity(tempvel);
            flywheel2.setVelocity(tempvel);

//            packet.put("1.target flywheel", velocity);
//            packet.put("2.velocity flywheel", flywheel.getVelocity());
//            packet.put("3.error flywheel", Math.abs( flywheel.getVelocity() - velocity ));
////            packet.put("mode", "velocity");
//            packet.put("4.OUTPUT", Math.abs( flywheel.getVelocity() - velocity ) > 15);

            if (Math.abs( flywheel1.getVelocity() - tarvel ) > 45) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 8) {
                velocityInitialized = false;
                return false;
            }

//            if (velocityTimer.milliseconds() > 6000) {
//                return false;
//            }

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

//            if (launchCloseTimer.milliseconds() >= launchTimerDelayToPassthrough_CLOSE) {
//                passthrough.setPower(PASSTHROUGH_POWER_CLOSE);
//            }

            if (launchCloseTimer.milliseconds() >= launchTimerDelayToIntake_CLOSE) {
                intake.setPower(INTAKE_POWER_CLOSE);
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

//            if (launchFarTimer.milliseconds() >= launchTimerDelayToPassthrough_FAR) {
//                passthrough.setPower(PASSTHROUGH_POWER_FAR);
//            }

            if (launchFarTimer.milliseconds() >= launchTimerDelayToIntake_FAR) {
                intake.setPower(INTAKE_POWER_FAR);
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
//                passthrough.setPower(0);
                intake.setPower(0);
                flywheel1.setPower(0);
                flywheel2.setPower(0);
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