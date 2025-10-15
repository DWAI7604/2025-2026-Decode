package org.firstinspires.ftc.teamcode.othershit;

import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="LimeLight", group="Linear OpMode")
public class LimeLight extends LinearOpMode {

    private State state = State.Pattern;

    private Limelight3A limelight;
    private Pattern pattern = Pattern.Unknown;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("status", "initialized");
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("status", "running");
            telemetry.addData("pattern", pattern);
            telemetry.addData("state", state);

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // decoding the pattern april tag and saving the pattern
                if (state == State.Pattern) {
                    pattern = decodePattern(result);

                    if (pattern != Pattern.Unknown) {
                        state = State.Next;
                        limelight.pipelineSwitch(1); // red tower
                    }
                }

                if (state == State.Next) { // turn towards red tower, move towards it
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ty = result.getTy(); // How far up or down the target is (degrees)
                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    // if (tx < 0) yaw = turn left
                    // if (tx > 0) yaw = turn right

                    telemetry.addData("Target X", tx);

                    int tposDisplay = (int) ( tx / 2 );
                    StringBuilder spaces1 = new StringBuilder();
                    StringBuilder spaces2 = new StringBuilder();
                    for (int i = 0; i < 10; i++) {
                        if (i < (tposDisplay + 5))
                            spaces1.append("_");
                        else
                            spaces2.append("_");
                    }
                    telemetry.addData("visual", "_____" + "X" + "_____");
                    telemetry.addData("visual", spaces1.toString() + "X" + spaces2.toString());

                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                }
//
//                // Access fiducial results
//                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                }

            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.update();
        }
    }

    // looks through the available april tags (limited to 21-23) and updates the pattern variable to match
    public Pattern decodePattern (LLResult result) { // fiducial == april tag

        Pattern pattern = Pattern.Unknown;

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            switch (fr.getFiducialId()) {
                case 21:
                    pattern = Pattern.GPP;
                    break;
                case 22:
                    pattern = Pattern.PGP;
                    break;
                case 23:
                    pattern = Pattern.PPG;
                    break;
            }
        }

        return pattern;
    }

    public enum Pattern {
        Unknown, GPP, PGP, PPG
    }

    public enum State {
        Pattern, Next
    }
}