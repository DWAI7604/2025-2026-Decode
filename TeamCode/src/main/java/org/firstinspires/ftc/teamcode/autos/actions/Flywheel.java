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

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder = hardwareMap.get(Servo.class, "feeder");
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
                flywheel.setPower(0.6);
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
            }

            if (timer2.milliseconds() < 150) {
                feeder.setPosition(.22);
            } else {
                feeder.setPosition(.001);
            }

            if (timer2.milliseconds() > 300) {
                return false;
            }

            return true;
        }
    }

    public Action power(double power, double seconds) {
        return new Power(power, seconds);
    }

    public Action launch() {
        return new Launch();
    }
}