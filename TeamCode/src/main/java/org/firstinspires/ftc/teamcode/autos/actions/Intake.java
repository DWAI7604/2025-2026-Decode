package org.firstinspires.ftc.teamcode.autos.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intake;
//    private CRServo transfer;
    private DcMotor transfer;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
//        transfer = hardwareMap.get(CRServo.class, "transfer");
        transfer = hardwareMap.get(DcMotor.class, "passthrough");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
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
                    intake.setPower(-1);
                else
                    intake.setPower(1);
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
                    transfer.setPower(-0.5);
                else
                    transfer.setPower(.5);
            }

            if (timer2.milliseconds() > milliseconds2) {
                transfer.setPower(0);
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
}