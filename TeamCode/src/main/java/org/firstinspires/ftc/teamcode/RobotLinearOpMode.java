package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


/**
 * This file is where each method should be written or referenced. Then when writing Autos and TeleOps,
 * programmers need to make files that extends RobotLinearOpMode instead of extending LinearOpMode.
 */

@Autonomous(name="RobotLinearOpMode", group="Linear Opmode")
@Disabled
public abstract class RobotLinearOpMode extends LinearOpMode {

    // Construction //
    public RobotLinearOpMode() {

    }

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;
    NormalizedColorSensor colorSensor;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    boolean USE_WEBCAM = false;  // true for webcam, false for phone camera
    boolean placingPixel = false;
    boolean searching;
    boolean aTagSeen = false;
    private ElapsedTime runtime = new ElapsedTime();










    public void encoderDrive(double power, double inches, MOVEMENT_DIRECTION movement_direction) {


        //Specifications of hardware
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double GEAR_RATIO = 19.2;
        final double COUNTS_PER_ROTATION_AT_MOTOR = 537.7;
        final double TICKS_PER_ROTATION = (COUNTS_PER_ROTATION_AT_MOTOR);
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);

        //Target # of ticks for each motor
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        //Resets motor encoders to 0 ticks
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightFrontTarget = rightFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        leftBackTarget = leftBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightBackTarget = rightBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);

        if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget - (int)(rightFrontTarget * 0.0016605117));
            leftBackDriveMotor.setTargetPosition(leftBackTarget);
            rightBackDriveMotor.setTargetPosition(rightBackTarget - (int)(rightBackTarget * 0.0016605117));

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftBackDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);


        }

        if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget );
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget + (int)(rightFrontTarget * 0.0016605117));
            leftBackDriveMotor.setTargetPosition(-leftBackTarget);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget + (int)(rightBackTarget * 0.0016605117));

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (rightFrontDriveMotor.isBusy() && opModeIsActive()) {

            }
            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget * 2 );
            rightFrontDriveMotor.setTargetPosition(2*(-rightFrontTarget + (int)(rightFrontTarget * 0.0016605117)));
            leftBackDriveMotor.setTargetPosition(leftBackTarget * 2 );
            rightBackDriveMotor.setTargetPosition((rightBackTarget - (int)(rightBackTarget * 0.0016605117))*2);


            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (rightFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget * 2);
            rightFrontDriveMotor.setTargetPosition((rightFrontTarget - (int)(rightFrontTarget * 0.0016605117))*2);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget * 2);
            rightBackDriveMotor.setTargetPosition(-(rightBackTarget + (int)(rightBackTarget * 0.0016605117))* 2);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (rightFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Kills the motors to prepare for next call of method
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }

    /*public void encoderLift(double power, double inches, LIFT_DIRECTION lift_direction) {

        //Specifications of hardware
        final double wheelDiameter = 1.5;
        final double wheelCircumference = (wheelDiameter * 3.141592653589793);
        final double ticksPerRotation = 28;
        final double ticksPerInch = (ticksPerRotation / wheelCircumference);

        int liftTarget;


        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTarget = leftLifter.getCurrentPosition() + (int) (inches * ticksPerInch);


        if (lift_direction == LIFT_DIRECTION.UP) {
            leftLifter.setTargetPosition(liftTarget);
            rightLifter.setTargetPosition(liftTarget);

            leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftLifter.setPower(power);
            rightLifter.setPower(power);


            while (leftLifter.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftLifter.setPower(0);
            rightLifter.setPower(0);

        }
    }*/

    public void encoderTurn(double power, double degrees, TURN_DIRECTION turn_direction) {





        //Declaration of important variables
//        final double WHEEL_DIAMETER_INCHES = 3.77953;
//        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
//        final double DRIVE_GEAR_REDUCTION = 1.0;
//        final double COUNTS_PER_ROTATION_AT_MOTOR = 537.7;
//        final double TICKS_PER_ROTATION = (COUNTS_PER_ROTATION_AT_MOTOR);
//
//        final double TICKS_PER_INCH = (TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE_INCHES);
//        //Measure in inches
//        final double ROBOT_LENGTH = 13;
//        final double ROBOT_WIDTH = (15 +(7/8));
//        //Uses pythagorean theorem to find the radius from the center of the robot to a point on its turning circle and subsequently the circumference of said circle
//        final double ROBOT_DIAMETER = (Math.sqrt(Math.pow(ROBOT_LENGTH, 2) + Math.pow(ROBOT_WIDTH, 2)));
//        final double ROBOT_CIRCUMFERENCE = (ROBOT_DIAMETER * 3.141592653589793);
//        //Finds the number of degrees each tick covers
//        final double INCHES_PER_DEGREE = (ROBOT_CIRCUMFERENCE/360);
//        final double TICKS_PER_DEGREE = (TICKS_PER_INCH * INCHES_PER_DEGREE);

        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double TICKS_PER_ROTATION = (537.7);
        final double ROBOT_LENGTH = 13;
        final double ROBOT_WIDTH = (15 +(7/8));
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE_INCHES);

        final double TURNING_DISTANCE = ((degrees/360) * 3.141592653589793 * Math.sqrt((Math.pow(ROBOT_LENGTH, 2)) + Math.pow(ROBOT_WIDTH, 2)));
        final double TICK_TARGET = TURNING_DISTANCE * TICKS_PER_INCH;





        declareHardwareProperties();

        //Target # of ticks for each motor
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Resets motor encoders to 0 ticks
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the taret # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDriveMotor.getCurrentPosition() + (int) (TICK_TARGET);
        rightFrontTarget = rightFrontDriveMotor.getCurrentPosition() + (int) (TICK_TARGET);
        leftBackTarget = leftBackDriveMotor.getCurrentPosition() + (int) (TICK_TARGET);
        rightBackTarget = rightBackDriveMotor.getCurrentPosition() + (int) (TICK_TARGET);

        if(turn_direction == TURN_DIRECTION.TURN_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(leftBackTarget);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (rightFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if(turn_direction == TURN_DIRECTION.TURN_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget );
            rightBackDriveMotor.setTargetPosition(rightBackTarget );

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (rightFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);

        }

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initAprilTag() {

        USE_WEBCAM = true;
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    public boolean[] getAprilTags()
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        boolean[] aprilTags = new boolean[7];
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null)
            {
                aprilTags[detection.id] = true;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        return aprilTags;
    }// end method telemetryAprilTag

    public void aprilTagCentering() {

        List<AprilTagDetection> currentDetection = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetection)
        {
            if (detection.metadata != null)
            {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }



        }   // end for() loop



    }




    public void colorSensorDrive(double power, MOVEMENT_DIRECTION movement_direction, TAPE_COLOR tape_color) {
        if (tape_color == TAPE_COLOR.RED_TAPE) {
            if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                while (opModeIsActive() && colorSensor() != 2) {
                    leftFrontDriveMotor.setPower(power);
                    leftBackDriveMotor.setPower(power);
                    rightFrontDriveMotor.setPower(power);
                    rightBackDriveMotor.setPower(power);

                    if (colorSensor() == 2) {
                        motorKill();
                        encoderDrive(power, .5, MOVEMENT_DIRECTION.REVERSE);
                    }

                }
            } else if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
                while (opModeIsActive() && colorSensor() != 2) {
                    leftFrontDriveMotor.setPower(-power);
                    leftBackDriveMotor.setPower(-power);
                    rightFrontDriveMotor.setPower(-power);
                    rightBackDriveMotor.setPower(-power);

                    if (colorSensor() == 2) {
                        motorKill();
                        encoderDrive(power, .5, MOVEMENT_DIRECTION.FORWARD);
                    }
                }
            } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {
                while (opModeIsActive() && colorSensor() != 2) {
                    leftFrontDriveMotor.setPower(power);
                    leftBackDriveMotor.setPower(-power);
                    rightFrontDriveMotor.setPower(power);
                    rightBackDriveMotor.setPower(-power);

                    if (colorSensor() == 2) {
                        motorKill();
                    }
                }
            } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {
                while (opModeIsActive() && colorSensor() != 2) {
                    leftFrontDriveMotor.setPower(-power);
                    leftBackDriveMotor.setPower(power);
                    rightFrontDriveMotor.setPower(-power);
                    rightBackDriveMotor.setPower(power);

                    if (colorSensor() == 2) {
                        motorKill();
                    }
                }
            }
        } else if (tape_color == TAPE_COLOR.BLUE_TAPE) {
            if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                while (opModeIsActive() && colorSensor() != 1) {
                    leftFrontDriveMotor.setPower(power);
                    leftBackDriveMotor.setPower(power);
                    rightFrontDriveMotor.setPower(power);
                    rightBackDriveMotor.setPower(power);

                    if (colorSensor() == 1) {
                        motorKill();
                        encoderDrive(power, .5, MOVEMENT_DIRECTION.REVERSE);
                    }
                }
            } else if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
                while (opModeIsActive() && colorSensor() != 1) {
                    leftFrontDriveMotor.setPower(-power);
                    leftBackDriveMotor.setPower(-power);
                    rightFrontDriveMotor.setPower(-power);
                    rightBackDriveMotor.setPower(-power);

                    if (colorSensor() == 1) {
                        motorKill();
                        encoderDrive(power, .5, MOVEMENT_DIRECTION.FORWARD);
                    }
                }
            } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {
                while (opModeIsActive() && colorSensor() != 1) {
                    leftFrontDriveMotor.setPower(power);
                    leftBackDriveMotor.setPower(-power);
                    rightFrontDriveMotor.setPower(power);
                    rightBackDriveMotor.setPower(-power);

                    if (colorSensor() == 1) {
                        motorKill();
                    }
                }
            } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {
                while (opModeIsActive() && colorSensor() !=1) {
                    leftFrontDriveMotor.setPower(-power);
                    leftBackDriveMotor.setPower(power);
                    rightFrontDriveMotor.setPower(-power);
                    rightBackDriveMotor.setPower(power);

                    if (colorSensor() == 1) {
                        motorKill();
                    }
                }
            }
        }
    }

    public double distanceSensor(SENSOR_DIRECTION sensor_direction) {
        DistanceSensor sensorDistance = null;
        double distance;

        if (sensor_direction == SENSOR_DIRECTION.REAR) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_rear");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;


        } else if (sensor_direction == SENSOR_DIRECTION.FRONT) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_front");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;


        } else if (sensor_direction == SENSOR_DIRECTION.LEFT) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;


        } else if (sensor_direction == SENSOR_DIRECTION.RIGHT) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        }
        // you can use this as a regular DistanceSensor.
        return sensorDistance.getDistance(DistanceUnit.INCH);



        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

    }
    public void distSensorDrive(double inputPower, double distanceFromObjectCM, MOVEMENT_DIRECTION movement_direction) {

        double power;
        double dist = distanceSensor(SENSOR_DIRECTION.REAR);
        if (dist > 10) {
            dist = 10;
        }

//        if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
//
//            while (opModeIsActive() && distanceSensor(SENSOR_DIRECTION.REAR) > distanceFromObjectCM) {
//                double distanceToSpeed = (Math.atan(distanceSensor(SENSOR_DIRECTION.REAR) - distanceFromObjectCM))/4;
//                if (distanceToSpeed > inputPower) {
//                    power = inputPower;
//                } else {
//                    power = distanceToSpeed;
//                }
//                leftFrontDriveMotor.setPower(-power);
//                rightFrontDriveMotor.setPower(-power);
//                leftBackDriveMotor.setPower(-power);
//                rightBackDriveMotor.setPower(-power);
//
//                if (distanceSensor(SENSOR_DIRECTION.REAR) <= distanceFromObjectCM) {
//                    motorKill();
//                }
//                telemetry.addData("Power", power);
//                telemetry.update();
//            }
        encoderDrive(inputPower, dist - distanceFromObjectCM, MOVEMENT_DIRECTION.REVERSE);
        encoderDrive(.1, 1, MOVEMENT_DIRECTION.REVERSE);



//                    leftFrontDriveMotor.setPower(-power);
//                    rightFrontDriveMotor.setPower(-power);
//                    leftBackDriveMotor.setPower(-power);
//                    rightBackDriveMotor.setPower(-power);
//                    sleep(200);
//                } while (distanceFromObjectCM > distanceSensor(SENSOR_DIRECTION.REAR));


//            while (distanceFromObjectCM >= distanceSensor(SENSOR_DIRECTION.REAR)) {
//
//                double power = Math.log(distanceSensor(SENSOR_DIRECTION.REAR));
//                if (power > inputPower) {
//                    power = inputPower;
//                }
//                leftFrontDriveMotor.setPower(power);
//                rightFrontDriveMotor.setPower(power);
//                leftBackDriveMotor.setPower(power);
//                rightBackDriveMotor.setPower(power);
//                sleep(50);
//            }
//            if (distanceFromObjectCM <= distanceSensor(SENSOR_DIRECTION.REAR)) {
//                motorKill();
//            }
    }



    public float colorSensor() {
        int colorValue = 0;



        View relativeLayout;


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 10;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        //final float[] hsvValues = new float[3];



        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }



        // Loop until we are asked to stop





        // Show the gain value via telemetry
        //telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);


        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
//            Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine();
//                    .addData("Hue", "%.3f", hsvValues[0])
//                    .addData("Saturation", "%.3f", hsvValues[1])
//                    .addData("Value", "%.3f", hsvValues[2]);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
//            if (colorSensor instanceof DistanceSensor) {
//                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
//            }

        if (colors.blue >= .080) {
            colorValue = 1;
        } else if (colors.red > .070) {
            colorValue = 2;
        }

        return(colorValue);
    }

    public void purplePixelPlace() {

        Servo purplePlacer;

        purplePlacer = hardwareMap.get(Servo.class, "purplePlacer");

        purplePlacer.setPosition(100);
        sleep(500);
        purplePlacer.setPosition(0);




    }

    public void yellowPixelPlace() {

        Servo yellowPlacer = null;

        yellowPlacer = hardwareMap.get(Servo.class, "yellowPlacer");


        yellowPlacer.setPosition(355);
        sleep(600);
        yellowPlacer.setPosition(0);
        sleep(400);

    }

//    public void cameraCodeBlue() {
//        OpenCvInternalCamera phoneCam;
//        SkystoneDeterminationExample.SkystoneDeterminationPipeline pipelineBlue;
//
//
//        waitForStart();
//        /**
//         * NOTE: Many comments have been omitted from this sample for the
//         * sake of conciseness. If you're just starting out with EasyOpenCv,
//         * you should take a look at {@link InternalCamera1Example} or its
//         * webcam counterpart, {@link WebcamExample} first.
//         */
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipelineBlue = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
//        phoneCam.setPipeline(pipelineBlue);
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Analysis", pipelineBlue.getAnalysis());
//            telemetry.update();
//
//
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
//
//    }
//        public static class imgPipelineBlue extends OpenCvPipeline
//        {
//            /*
//             * An enum to define the skystone position
//             */
//            public enum SkystonePosition
//            {
//                LEFT,
//                CENTER,
//                RIGHT
//            }
//
//            /*
//             * Some color constants
//             */
//            static final Scalar BLUE = new Scalar(0, 0, 255);
//            static final Scalar GREEN = new Scalar(0, 255, 0);
//
//            /*
//             * The core values which define the location and size of the sample regions
//             */
//            static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,120);
//            static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(107,120);
//            static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(213,120);
//            static final int REGION_WIDTH = 106;
//            static final int REGION_HEIGHT = 60;
//
//            /*
//             * Points which actually define the sample region rectangles, derived from above values
//             *
//             * Example of how points A and B work to define a rectangle
//             *
//             *   ------------------------------------
//             *   | (0,0) Point A                    |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                  Point B (70,50) |
//             *   ------------------------------------
//             *
//             */
//            Point region1_pointA = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y);
//            Point region1_pointB = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//            Point region2_pointA = new Point(
//                    REGION2_TOPLEFT_ANCHOR_POINT.x,
//                    REGION2_TOPLEFT_ANCHOR_POINT.y);
//            Point region2_pointB = new Point(
//                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//            Point region3_pointA = new Point(
//                    REGION3_TOPLEFT_ANCHOR_POINT.x,
//                    REGION3_TOPLEFT_ANCHOR_POINT.y);
//            Point region3_pointB = new Point(
//                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//            /*
//             * Working variables
//             */
//            Mat region1_Cb, region2_Cb, region3_Cb;
//            Mat YCrCb = new Mat();
//            Mat Cb = new Mat();
//            int avg1, avg2, avg3;
//
//            // Volatile since accessed by OpMode thread w/o synchronization
//            private volatile SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition positionBlue = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT;
//
//            /*
//             * This function takes the RGB frame, converts to YCrCb,
//             * and extracts the Cb channel to the 'Cb' variable
//             */
//            void inputToCb(Mat input)
//            {
//                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//                Core.extractChannel(YCrCb, Cb, 2);
//            }
//
//            @Override
//            public void init(Mat firstFrame)
//            {
//                /*
//                 * We need to call this in order to make sure the 'Cb'
//                 * object is initialized, so that the submats we make
//                 * will still be linked to it on subsequent frames. (If
//                 * the object were to only be initialized in processFrame,
//                 * then the submats would become delinked because the backing
//                 * buffer would be re-allocated the first time a real frame
//                 * was crunched)
//                 */
//                inputToCb(firstFrame);
//
//                /*
//                 * Submats are a persistent reference to a region of the parent
//                 * buffer. Any changes to the child affect the parent, and the
//                 * reverse also holds true.
//                 */
//                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//                region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//                region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
//            }
//
//            @Override
//            public Mat processFrame(Mat input)
//            {
//                /*
//                 * Overview of what we're doing:
//                 *
//                 * We first convert to YCrCb color space, from RGB color space.
//                 * Why do we do this? Well, in the RGB color space, chroma and
//                 * luma are intertwined. In YCrCb, chroma and luma are separated.
//                 * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
//                 * are Y, the luma channel (which essentially just a B&W image), the
//                 * Cr channel, which records the difference from red, and the Cb channel,
//                 * which records the difference from blue. Because chroma and luma are
//                 * not related in YCrCb, vision code written to look for certain values
//                 * in the Cr/Cb channels will not be severely affected by differing
//                 * light intensity, since that difference would most likely just be
//                 * reflected in the Y channel.
//                 *
//                 * After we've converted to YCrCb, we extract just the 2nd channel, the
//                 * Cb channel. We do this because stones are bright yellow and contrast
//                 * STRONGLY on the Cb channel against everything else, including SkyStones
//                 * (because SkyStones have a black label).
//                 *
//                 * We then take the average pixel value of 3 different regions on that Cb
//                 * channel, one positioned over each stone. The brightest of the 3 regions
//                 * is where we assume the SkyStone to be, since the normal stones show up
//                 * extremely darkly.
//                 *
//                 * We also draw rectangles on the screen showing where the sample regions
//                 * are, as well as drawing a solid rectangle over top the sample region
//                 * we believe is on top of the SkyStone.
//                 *
//                 * In order for this whole process to work correctly, each sample region
//                 * should be positioned in the center of each of the first 3 stones, and
//                 * be small enough such that only the stone is sampled, and not any of the
//                 * surroundings.
//                 */
//
//                /*
//                 * Get the Cb channel of the input frame after conversion to YCrCb
//                 */
//                inputToCb(input);
//
//                /*
//                 * Compute the average pixel value of each submat region. We're
//                 * taking the average of a single channel buffer, so the value
//                 * we need is at index 0. We could have also taken the average
//                 * pixel value of the 3-channel image, and referenced the value
//                 * at index 2 here.
//                 */
//                avg1 = (int) Core.mean(region1_Cb).val[0];
//                avg2 = (int) Core.mean(region2_Cb).val[0];
//                avg3 = (int) Core.mean(region3_Cb).val[0];
//
//                /*
//                 * Draw a rectangle showing sample region 1 on the screen.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//                /*
//                 * Draw a rectangle showing sample region 2 on the screen.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region2_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//                /*
//                 * Draw a rectangle showing sample region 3 on the screen.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region3_pointA, // First point which defines the rectangle
//                        region3_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//
//                /*
//                 * Find the max of the 3 averages
//                 */
//                int maxOneTwo = Math.max(avg1, avg2);
//                int max = Math.max(maxOneTwo, avg3);
//
//                /*
//                 * Now that we found the max, we actually need to go and
//                 * figure out which sample region that value was from
//                 */
//                if(max == avg1) // Was it from region 1?
//                {
//                    positionBlue = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // Record our analysis
//
//                    /*
//                     * Draw a solid rectangle on top of the chosen region.
//                     * Simply a visual aid. Serves no functional purpose.
//                     */
//                    Imgproc.rectangle(
//                            input, // Buffer to draw on
//                            region1_pointA, // First point which defines the rectangle
//                            region1_pointB, // Second point which defines the rectangle
//                            GREEN, // The color the rectangle is drawn in
//                            -1); // Negative thickness means solid fill
//                }
//                else if(max == avg2) // Was it from region 2?
//                {
//                    positionBlue = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.CENTER; // Record our analysis
//
//                    /*
//                     * Draw a solid rectangle on top of the chosen region.
//                     * Simply a visual aid. Serves no functional purpose.
//                     */
//                    Imgproc.rectangle(
//                            input, // Buffer to draw on
//                            region2_pointA, // First point which defines the rectangle
//                            region2_pointB, // Second point which defines the rectangle
//                            GREEN, // The color the rectangle is drawn in
//                            -1); // Negative thickness means solid fill
//                }
//                else if(max == avg3) // Was it from region 3?
//                {
//                    positionBlue = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT; // Record our analysis
//
//                    /*
//                     * Draw a solid rectangle on top of the chosen region.
//                     * Simply a visual aid. Serves no functional purpose.
//                     */
//                    Imgproc.rectangle(
//                            input, // Buffer to draw on
//                            region3_pointA, // First point which defines the rectangle
//                            region3_pointB, // Second point which defines the rectangle
//                            GREEN, // The color the rectangle is drawn in
//                            -1); // Negative thickness means solid fill
//                }
//
//                /*
//                 * Render the 'input' buffer to the viewport. But note this is not
//                 * simply rendering the raw camera feed, because we called functions
//                 * to add some annotations to this buffer earlier up.
//                 */
//                return input;
//            }
//
//
//            /*
//             * Call this from the OpMode thread to obtain the latest analysis
//             */
//            public SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition getAnalysis()
//            {
//                return positionBlue;
//            }
//        }
//
//        public void cameraCodeRed() {
//            OpenCvInternalCamera phoneCam;
//            SkystoneDeterminationExample.SkystoneDeterminationPipeline pipelineRed;
//
//
//            waitForStart();
//            /**
//             * NOTE: Many comments have been omitted from this sample for the
//             * sake of conciseness. If you're just starting out with EasyOpenCv,
//             * you should take a look at {@link InternalCamera1Example} or its
//             * webcam counterpart, {@link WebcamExample} first.
//             */
//
//            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//            pipelineRed = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
//            phoneCam.setPipeline(pipelineRed);
//
//            // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//            // out when the RC activity is in portrait. We do our actual image processing assuming
//            // landscape orientation, though.
//            phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//                    /*
//                     * This will be called if the camera could not be opened
//                     */
//                }
//            });
//            waitForStart();
//
//            while (opModeIsActive()) {
//                telemetry.addData("Analysis", pipelineRed.getAnalysis());
//                telemetry.update();
//
//                // Don't burn CPU cycles busy-looping in this sample
//                sleep(50);
//            }
//        }
//        public static class imgPipelineRed extends OpenCvPipeline {
//            /*
//             * An enum to define the skystone position
//             */
//            public enum SkystonePosition
//            {
//                LEFT,
//                CENTER,
//                RIGHT
//            }
//
//            /*
//             * Some color constants
//             */
//            static final Scalar BLUE = new Scalar(0, 0, 255);
//            static final Scalar GREEN = new Scalar(0, 255, 0);
//
//            /*
//             * The core values which define the location and size of the sample regions
//             */
//            static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,120);
//            static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(107,120);
//            static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(213,120);
//            static final int REGION_WIDTH = 106;
//            static final int REGION_HEIGHT = 60;
//
//            /*
//             * Points which actually define the sample region rectangles, derived from above values
//             *
//             * Example of how points A and B work to define a rectangle
//             *
//             *   ------------------------------------
//             *   | (0,0) Point A                    |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                                  |
//             *   |                  Point B (70,50) |
//             *   ------------------------------------
//             *
//             */
//            Point region1_pointA = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y);
//            Point region1_pointB = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//            Point region2_pointA = new Point(
//                    REGION2_TOPLEFT_ANCHOR_POINT.x,
//                    REGION2_TOPLEFT_ANCHOR_POINT.y);
//            Point region2_pointB = new Point(
//                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//            Point region3_pointA = new Point(
//                    REGION3_TOPLEFT_ANCHOR_POINT.x,
//                    REGION3_TOPLEFT_ANCHOR_POINT.y);
//            Point region3_pointB = new Point(
//                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//            /*
//             * Working variables
//             */
//            Mat region1_Cb, region2_Cb, region3_Cb;
//            Mat YCrCb = new Mat();
//            Mat Cb = new Mat();
//            int avg1, avg2, avg3;
//
//            // Volatile since accessed by OpMode thread w/o synchronization
//            private volatile SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition positionRed = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT;
//
//            /*
//             * This function takes the RGB frame, converts to YCrCb,
//             * and extracts the Cb channel to the 'Cb' variable
//             */
//            void inputToCb(Mat input)
//            {
//                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//                Core.extractChannel(YCrCb, Cb, 2);
//            }
//
//            @Override
//            public void init(Mat firstFrame)
//            {
//                /*
//                 * We need to call this in order to make sure the 'Cb'
//                 * object is initialized, so that the submats we make
//                 * will still be linked to it on subsequent frames. (If
//                 * the object were to only be initialized in processFrame,
//                 * then the submats would become delinked because the backing
//                 * buffer would be re-allocated the first time a real frame
//                 * was crunched)
//                 */
//                inputToCb(firstFrame);
//
//                /*
//                 * Submats are a persistent reference to a region of the parent
//                 * buffer. Any changes to the child affect the parent, and the
//                 * reverse also holds true.
//                 */
//                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//                region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//                region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
//            }
//
//            @Override
//            public Mat processFrame(Mat input)
//            {
//                /*
//                 * Overview of what we're doing:
//                 *
//                 * We first convert to YCrCb color space, from RGB color space.
//                 * Why do we do this? Well, in the RGB color space, chroma and
//                 * luma are intertwined. In YCrCb, chroma and luma are separated.
//                 * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
//                 * are Y, the luma channel (which essentially just a B&W image), the
//                 * Cr channel, which records the difference from red, and the Cb channel,
//                 * which records the difference from blue. Because chroma and luma are
//                 * not related in YCrCb, vision code written to look for certain values
//                 * in the Cr/Cb channels will not be severely affected by differing
//                 * light intensity, since that difference would most likely just be
//                 * reflected in the Y channel.
//                 *
//                 * After we've converted to YCrCb, we extract just the 2nd channel, the
//                 * Cb channel. We do this because stones are bright yellow and contrast
//                 * STRONGLY on the Cb channel against everything else, including SkyStones
//                 * (because SkyStones have a black label).
//                 *
//                 * We then take the average pixel value of 3 different regions on that Cb
//                 * channel, one positioned over each stone. The brightest of the 3 regions
//                 * is where we assume the SkyStone to be, since the normal stones show up
//                 * extremely darkly.
//                 *
//                 * We also draw rectangles on the screen showing where the sample regions
//                 * are, as well as drawing a solid rectangle over top the sample region
//                 * we believe is on top of the SkyStone.
//                 *
//                 * In order for this whole process to work correctly, each sample region
//                 * should be positioned in the center of each of the first 3 stones, and
//                 * be small enough such that only the stone is sampled, and not any of the
//                 * surroundings.
//                 */
//
//                /*
//                 * Get the Cb channel of the input frame after conversion to YCrCb
//                 */
//                inputToCb(input);
//
//                /*
//                 * Compute the average pixel value of each submat region. We're
//                 * taking the average of a single channel buffer, so the value
//                 * we need is at index 0. We could have also taken the average
//                 * pixel value of the 3-channel image, and referenced the value
//                 * at index 2 here.
//                 */
//                avg1 = (int) Core.mean(region1_Cb).val[0];
//                avg2 = (int) Core.mean(region2_Cb).val[0];
//                avg3 = (int) Core.mean(region3_Cb).val[0];
//
//                /*
//                 * Draw a rectangle showing sample region 1 on the screen.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//                /*
//                 * Draw a rectangle showing sample region 2 on the screen.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region2_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//                /*
//                 * Draw a rectangle showing sample region 3 on the screen.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region3_pointA, // First point which defines the rectangle
//                        region3_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//
//                /*
//                 * Find the max of the 3 averages
//                 */
//                int minOneTwo = Math.min(avg1, avg2);
//                int min = Math.min(minOneTwo, avg3);
//
//                /*
//                 * Now that we found the max, we actually need to go and
//                 * figure out which sample region that value was from
//                 */
//                if(min == avg1) // Was it from region 1?
//                {
//                    positionRed = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // Record our analysis
//
//                    /*
//                     * Draw a solid rectangle on top of the chosen region.
//                     * Simply a visual aid. Serves no functional purpose.
//                     */
//                    Imgproc.rectangle(
//                            input, // Buffer to draw on
//                            region1_pointA, // First point which defines the rectangle
//                            region1_pointB, // Second point which defines the rectangle
//                            GREEN, // The color the rectangle is drawn in
//                            -1); // Negative thickness means solid fill
//                }
//                else if(min == avg2) // Was it from region 2?
//                {
//                    positionRed = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.CENTER; // Record our analysis
//
//                    /*
//                     * Draw a solid rectangle on top of the chosen region.
//                     * Simply a visual aid. Serves no functional purpose.
//                     */
//                    Imgproc.rectangle(
//                            input, // Buffer to draw on
//                            region2_pointA, // First point which defines the rectangle
//                            region2_pointB, // Second point which defines the rectangle
//                            GREEN, // The color the rectangle is drawn in
//                            -1); // Negative thickness means solid fill
//                }
//                else if(min == avg3) // Was it from region 3?
//                {
//                    positionRed = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT; // Record our analysis
//
//                    /*
//                     * Draw a solid rectangle on top of the chosen region.
//                     * Simply a visual aid. Serves no functional purpose.
//                     */
//                    Imgproc.rectangle(
//                            input, // Buffer to draw on
//                            region3_pointA, // First point which defines the rectangle
//                            region3_pointB, // Second point which defines the rectangle
//                            GREEN, // The color the rectangle is drawn in
//                            -1); // Negative thickness means solid fill
//                }
//
//                /*
//                 * Render the 'input' buffer to the viewport. But note this is not
//                 * simply rendering the raw camera feed, because we called functions
//                 * to add some annotations to this buffer earlier up.
//                 */
//                return input;
//            }
//
//            /*
//             * Call this from the OpMode thread to obtain the latest analysis
//             */
//            public SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition getAnalysis()
//            {
//                return positionRed;
//            }
//        }


    public void motorKill() {
        //Kills the motors to prepare for next call of method
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }

    public void declareHardwareProperties() {


        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");

        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    }

    enum MOVEMENT_DIRECTION {
        STRAFE_LEFT,
        STRAFE_RIGHT,
        FORWARD,
        REVERSE,
    }

    enum TURN_DIRECTION {
        TURN_LEFT,
        TURN_RIGHT

    }

    enum LIFT_DIRECTION {
        DOWN,
        UP
    }

    enum SENSOR_DIRECTION {
        FRONT,
        REAR,
        LEFT,
        RIGHT

    }

    enum TAPE_COLOR {
        RED_TAPE,
        BLUE_TAPE
    }



}