package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public class Launch implements Action {
        private boolean initialized2 = false;
        private ElapsedTime timer2 = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized2) {
                initialized2 = true;
                timer2.reset();
                feeder.setPosition(.001);
            }

            if (timer2.milliseconds() < 1000) {
                feeder.setPosition(.001);
                packet.put("target: ", "0.001");
            } else {
                feeder.setPosition(.22);
                packet.put("target: ", "0.22");
            }

            packet.put("position: ", feeder.getPosition());

            if (timer2.milliseconds() > 2000) {
                return false;
            }

            return true;
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

    public Action angle(double angle) {
        return new Angle(angle);
    }
}