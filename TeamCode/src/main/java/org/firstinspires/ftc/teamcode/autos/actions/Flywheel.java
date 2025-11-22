package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private DcMotorEx flywheel;
    private Servo feeder;
    private Servo angleAdjustServo;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder = hardwareMap.get(Servo.class, "feeder");
        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");
    }

    public class Power implements Action {
        private boolean initialized1 = false;
        private ElapsedTime timer1 = new ElapsedTime();
        double power;
        double seconds;

        public Power(double power, double seconds) {
            this.power = power;
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized1) {
                flywheel.setPower(power);
                initialized1 = true;
                timer1.reset();
            }

            return timer1.seconds() < seconds;
        }
    }

    public class Velocity implements Action {
        private boolean initialized4 = false;
        double velocity;
        int numOfCorrect = 0;

        public Velocity(double velocity) {
            this.velocity = velocity;
            initialized4 = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized4) {
                flywheel.setVelocity(velocity);
                initialized4 = true;
            }

            flywheel.setVelocity(velocity);

            packet.put("1.target flywheel", velocity);
            packet.put("2.velocity flywheel", flywheel.getVelocity());
            packet.put("3.error flywheel", Math.abs( flywheel.getVelocity() - velocity ));
//            packet.put("mode", "velocity");
            packet.put("4.OUTPUT", Math.abs( flywheel.getVelocity() - velocity ) > 15);

            if (Math.abs( flywheel.getVelocity() - velocity ) > 15) {
                numOfCorrect = 0;
            } else {
                numOfCorrect++;
            }

            if (numOfCorrect >= 3) {
                initialized4 = false;
                return false;
            }
            return true;
        }
    }

    public class Launch implements Action {
        private boolean initialized2 = false;
        private ElapsedTime timer2 = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized2) {
                initialized2 = true;
                timer2.reset();
                feeder.setPosition(.2);
            }
            packet.put("mode", "launch");

            if (timer2.milliseconds() < 400) {
                feeder.setPosition(.24);
                packet.put("target: ", "0.24");
            } else {
                feeder.setPosition(.67);
                packet.put("target: ", "0.67");
            }

            packet.put("position: ", feeder.getPosition());

            return timer2.milliseconds() < 700;
        }
    }

    public class PrepLaunch implements Action {
        private boolean initialized6 = false;
        private ElapsedTime timer6 = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized6) {
                initialized6 = true;
                timer6.reset();
                feeder.setPosition(.4);
            }
            packet.put("mode", "prep-launch");

            if (timer6.milliseconds() < 400) {
                feeder.setPosition(.65);
                packet.put("target: ", "0.65");
            } else {
                feeder.setPosition(.67);
                packet.put("target: ", "0.67");
            }

            packet.put("position: ", feeder.getPosition());

            return timer6.milliseconds() < 700;
        }
    }

    public class Angle implements Action {
        private boolean initialized3 = false;
        private ElapsedTime timer3 = new ElapsedTime();
        private double angle;

        public Angle(double angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized3) {
                initialized3 = true;
                timer3.reset();
                angleAdjustServo.setPosition(angle);
            }

            return timer3.milliseconds() < 200;
        }
    }

    public Action power(double power, double seconds) {
        return new Power(power, seconds);
    }

    public Action launch() {
        return new Launch();
    }

    public Action prepLaunch() {
        return new PrepLaunch();
    }

    public Action angle(double angle) {
        return new Angle(angle);
    }

    public Action velocity(double velocity) { return new Velocity(velocity); }
}