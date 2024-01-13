package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="NEW_RED_RIGHT", group="cvAutos")
//RED TOP STARTING PLACE
public class NEW_RED_RIGHT extends OpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot robot = new Robot();

    // HardwareMap hardwareMap;

    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;

    TouchSensor left_touch_sensor = null;
    TouchSensor right_touch_sensor = null;

    DcMotor left_slide_motor = null;
    DcMotor right_slide_motor = null;

    CRServo left_intake = null;
    CRServo right_intake = null;

    Servo left_claw_servo = null;
    Servo right_claw_servo = null;

    Servo left_slide_servo = null;
    Servo right_slide_servo = null;

    // Servo tilt = hardwareMap.servo.get("tilt");
    // Servo launch = hardwareMap.servo.get("launch");

    SampleMecanumDrive drive = null;

    camera pipeline;
    int beacon = 0;

    Pose2d startPos = null;
    Pose2d depositRight = null;
    Pose2d depositLeft = null;
    Pose2d depositCenter = null;
    Pose2d placeLeft = null;
    Pose2d placeCenter = null;
    Pose2d placeRight = null;
    Pose2d parking = null;

    boolean tSensorRising = false;

    OpenCvWebcam webcam;
    int cameraMonitorViewId = 0;

    TrajectorySequence purpleLeft = null;

    TrajectorySequence purpleMid = null;

    TrajectorySequence purpleRight = null;

    TrajectorySequence yellowLeft = null;

    TrajectorySequence yellowMid = null;

    TrajectorySequence yellowRight = null;

    TrajectorySequence parkFromMid = null;

    TrajectorySequence parkFromLeft = null;

    TrajectorySequence parkFromRight = null;

    int stage = 0;

    public void init() {
        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("front_right_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("back_right_motor");

        left_slide_motor = hardwareMap.dcMotor.get("left_slide_motor");
        right_slide_motor = hardwareMap.dcMotor.get("right_slide_motor");

        left_touch_sensor = hardwareMap.touchSensor.get("lefttouch");
        right_touch_sensor = hardwareMap.touchSensor.get("righttouch");

        left_intake = hardwareMap.crservo.get("left_intake");
        right_intake = hardwareMap.crservo.get("right_intake");

        left_claw_servo = hardwareMap.servo.get("left_claw_servo");
        right_claw_servo = hardwareMap.servo.get("right_claw_servo");

        left_slide_servo = hardwareMap.servo.get("left_slide_servo");
        right_slide_servo = hardwareMap.servo.get("right_slide_servo");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new camera();
        webcam.setPipeline(pipeline);

        robot.init(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        startPos = new Pose2d(65, 12, Math.toRadians(0));
        depositRight = new Pose2d(30, 12, Math.toRadians(0));
        depositLeft = new Pose2d(30, 12, Math.toRadians(270));
        depositCenter = new Pose2d(36, 12, Math.toRadians(180));
        placeLeft = new Pose2d(28, 54, Math.toRadians(90));
        placeCenter = new Pose2d(36, 56, Math.toRadians(90));
        placeRight = new Pose2d(44, 54, Math.toRadians(90));
        parking = new Pose2d(60, 50, Math.toRadians(180));

        purpleLeft = drive.trajectorySequenceBuilder(startPos)
                //.back(10)
                //.strafeLeft(10)
                .lineToLinearHeading(depositLeft)
                //.forward(10)
                //.turn(Math.toRadians(90))
                .build();

        purpleMid = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(depositCenter)
                //.turn(180)
                //.forward(15)
                .build();

        purpleRight = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(depositRight)
                //.turn(Math.toRadians(-90))
                //.forward(10)
                //.turn(Math.toRadians(-90))
                .build();

        yellowLeft = drive.trajectorySequenceBuilder(depositLeft)
                .lineToLinearHeading(placeLeft)
                .build();

        yellowMid = drive.trajectorySequenceBuilder(depositCenter)
                .forward(4)
                .lineToLinearHeading(placeCenter)
                .build();

        yellowRight = drive.trajectorySequenceBuilder(depositRight)
                .strafeLeft(24)
                .lineToLinearHeading(placeRight)
                .build();

        parkFromMid = drive.trajectorySequenceBuilder(placeCenter)
                .back(5)
                .lineToLinearHeading(parking)
                .build();

        parkFromLeft = drive.trajectorySequenceBuilder(placeLeft)
                .back(5)
                .lineToLinearHeading(parking)
                .build();

        parkFromRight = drive.trajectorySequenceBuilder(placeRight)
                .back(5)
                .lineToLinearHeading(parking)
                .build();

        drive.setPoseEstimate(startPos);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                  This will be called if the camera could not be opened
                */
            }
        });
    }

    public void init_loop() {
        if (pipeline.getRedAnalysis() == camera.SkystonePosition.RIGHT) {
            telemetry.addData("right", "found item in right box");
            telemetry.update();
            // drive.followTrajectorySequence(purpleRight);

            beacon = 2;
        }
        else if (pipeline.getRedAnalysis() == camera.SkystonePosition.LEFT) {
            telemetry.addData("left", "found item in left box");
            telemetry.update();
            // drive.followTrajectorySequence(purpleLeft);

            beacon = 0;
        }
        else if (pipeline.getRedAnalysis() == camera.SkystonePosition.CENTER) {
            telemetry.addData("center", "found item in center box");
            telemetry.update();
            // drive.followTrajectorySequence(purpleMid);

            beacon = 1;
        }

        drive.setPoseEstimate(startPos);

    }

    public void start() {

        //robot.init(hardwareMap, telemetry);
        left_slide_servo.setPosition(0.7);
        right_slide_servo.setPosition(0.4);
        left_claw_servo.setPosition(0.4);
        right_claw_servo.setPosition(0.3);

        startPos = new Pose2d(65, 12, Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        drive.followTrajectorySequenceAsync(purpleLeft);

    }

    public void loop() {

        /*if(left_touch_sensor.isPressed() && right_touch_sensor.isPressed() && !tSensorRising) {
            left_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tSensorRising = true;
        }
        if(left_touch_sensor.isPressed() && right_touch_sensor.isPressed() && tSensorRising) {
            left_slide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_slide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(!left_touch_sensor.isPressed() && !right_touch_sensor.isPressed() && tSensorRising) {
            tSensorRising = false;
        }*/
/*
        if(stage == 0) {
            startPos = new Pose2d(65, 12, Math.toRadians(0));
            drive.setPoseEstimate(startPos);

            telemetry.addData("Stage 0 complete: ", true);
            stage++;
        }

        if(stage == 1) {
            if(beacon == 0) {
                drive.followTrajectorySequence(purpleLeft);
            }
            if(beacon == 1) {
                drive.followTrajectorySequence(purpleMid);
            }
            if(beacon == 2) {
                drive.followTrajectorySequence(purpleRight);
            }
            if(!drive.isBusy()) {
                stage++;
            }
        }
*/
        /*
        else if(stage == 2) {
            // DROP PURPLE
            stage++;
        }
        else if(stage == 3) {
            if(beacon == 1) {drive.followTrajectorySequence(yellowMid);}
            if(beacon == 0) {drive.followTrajectorySequence(yellowLeft);}
            if(beacon == 2) {drive.followTrajectorySequence(yellowRight);}
            robot.setSlidesPos(1500);
            left_slide_servo.setPosition(0.45);
            if(!drive.isBusy()) {
                // DROP YELLOW
                stage++;
            }
        }
        else if(stage == 4) {
            if(beacon == 1) {drive.followTrajectorySequence(parkFromMid);}
            if(beacon == 0) {drive.followTrajectorySequence(parkFromLeft);}
            if(beacon == 2) {drive.followTrajectorySequence(parkFromRight);}
        }
        */
        drive.update();
    }

    public void stop() {

    }

}