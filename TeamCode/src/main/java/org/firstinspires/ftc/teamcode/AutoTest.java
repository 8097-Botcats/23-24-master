package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Red top w/ white wh", group="cvAutos")
//RED TOP STARTING PLACE
public class AutoTest extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0);

        TrajectorySequence placement = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .forward(12)
                .build();

        Robot robot = new Robot();

        robot.init(hardwareMap,telemetry);



        // OpenCvWebcam webcam;
        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // camera pipeline;

        // pipeline = new camera();
        // webcam.setPipeline(pipeline);

        // webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // @Override
            // public void onOpened() {
            //     webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
           //  }

           //  @Override
           //  public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
           //  }
        // });

        waitForStart();
        if (opModeIsActive()) {
            drive.followTrajectorySequence(placement);
        }
    }
}