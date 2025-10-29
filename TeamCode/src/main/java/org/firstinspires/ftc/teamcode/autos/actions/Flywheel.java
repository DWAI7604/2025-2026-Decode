package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private DcMotorEx flywheel;
    private Servo feeder;

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feeder = hardwareMap.get(Servo.class, "feeder");
    }

    public class Power implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        double power;
        double seconds;

        public Power(double power, double seconds) {
            this.power = power;
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flywheel.setPower(0.6);
                initialized = true;
                timer.reset();
            }

            return timer.seconds() < 0.5;
        }
    }

    public class Launch implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                timer.reset();
                feeder.setPosition(.22);
            }

            if (timer.seconds() > 0.5) {
                feeder.setPosition(.001);
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