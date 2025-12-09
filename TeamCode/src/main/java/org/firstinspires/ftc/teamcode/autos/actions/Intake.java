package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intake;
    private DcMotor passthrough;
    private TouchSensor limitSwitch;

    private static double INTAKE_POWER = 1.0;
    private static double PASSTHROUGH_POWER = 0.5;

    public static double limSwitchDelayToPassthrough = 250; // ms

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
        passthrough.setDirection(DcMotor.Direction.REVERSE);
        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
    }

    public class IntakeIn implements Action {
        private boolean initialized1 = false;
        private ElapsedTime timer1 = new ElapsedTime();
        private double milliseconds1;
        private boolean reversed = false;

        public IntakeIn(double milliseconds) {
            this.milliseconds1 = milliseconds;
        }
        public IntakeIn(double milliseconds, boolean reversed) {
            this.milliseconds1 = milliseconds;
            this.reversed = reversed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized1) {
                initialized1 = true;
                timer1.reset();

                if (reversed)
                    intake.setPower(-INTAKE_POWER);
                else
                    intake.setPower(INTAKE_POWER);
            }

            if (timer1.milliseconds() > milliseconds1) {
                intake.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class TransferIn implements Action {
        private boolean initialized2 = false;
        private ElapsedTime timer2 = new ElapsedTime();
        private double milliseconds2;
        private boolean reversed = false;

        public TransferIn(double milliseconds) {
            this.milliseconds2 = milliseconds;
        }
        public TransferIn(double milliseconds, boolean reversed) {
            this.milliseconds2 = milliseconds;
            this.reversed = reversed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized2) {
                initialized2 = true;
                timer2.reset();

                if (reversed)
                    passthrough.setPower(-PASSTHROUGH_POWER);
                else
                    passthrough.setPower(PASSTHROUGH_POWER);
            }

            if (timer2.milliseconds() > milliseconds2) {
                passthrough.setPower(0);
                return false;
            }

            return true;
        }
    }

    public class Passthrough implements Action {
        private boolean passthroughInitialized = false;
        private ElapsedTime passthroughTimer = new ElapsedTime();

        private boolean ending = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!passthroughInitialized) {
                passthroughInitialized = true;
                passthroughTimer.reset();

                passthrough.setPower(PASSTHROUGH_POWER);
            }

            if (!limitSwitch.isPressed() && !ending) { //  negative on lim switch because limit switch is wired backwards

                ending = true;

                passthroughTimer.reset();

            }

            if (ending && passthroughTimer.milliseconds() > limSwitchDelayToPassthrough) {
                passthrough.setPower(0);
                return false;
            }

            return true;
        }
    }

    public Action intakeIn(double milliseconds) {
        return new IntakeIn(milliseconds);
    }
    public Action intakeIn(double milliseconds, boolean reversed) {
        return new IntakeIn(milliseconds, reversed);
    }

    public Action transferIn(double milliseconds) {
        return new TransferIn(milliseconds);
    }
    public Action transferIn(double milliseconds, boolean reversed) {
        return new TransferIn(milliseconds, reversed);
    }

    public Action passthrough () { return new Passthrough(); }
}