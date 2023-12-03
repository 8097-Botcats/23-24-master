package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.Timer;

@Autonomous(name="COMP1 AUTO_BLUE", group="cvAutos")
//RED TOP STARTING PLACE
public class AutoTest2 extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx fl = null;
        DcMotorEx fr = null;
        DcMotorEx bl = null;
        DcMotorEx br = null;

        Timer timer = new Timer();

        fl = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        fr = hardwareMap.get(DcMotorEx.class,  "front_right_motor");
        bl = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        br = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        /* Pose2d startPose = new Pose2d(0, 0);

        TrajectorySequence placement = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .forward(12)
                .build(); */

        //Robot robot = new Robot();

        //robot.init(hardwareMap,telemetry);



        waitForStart();
        if (opModeIsActive()) {
            fl.setPower(0.5);
            // fr.setVelocity(1, AngleUnit.RADIANS);
            // bl.setVelocity(1, AngleUnit.RADIANS);
            // br.setVelocity(1, AngleUnit.RADIANS);
            sleep(10000);
            fl.setVelocity(0);
            fr.setVelocity(0);
            bl.setVelocity(0);
            br.setVelocity(0);
        }
    }
}