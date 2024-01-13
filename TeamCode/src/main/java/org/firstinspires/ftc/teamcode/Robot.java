package org.firstinspires.ftc.teamcode;



import static android.os.SystemClock.currentThreadTimeMillis;
import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

public class Robot {
    /* Public OpMode members. */
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    DcMotor left_slide_motor = null;
    DcMotor right_slide_motor = null;

    TouchSensor left_touch_sensor = null;
    TouchSensor right_touch_sensor = null;

    CRServo left_intake = null;
    CRServo right_intake = null;

    Servo left_claw_servo = null;
    Servo right_claw_servo = null;

    Servo left_slide_servo = null;
    Servo right_slide_servo = null;

    CRServo sternPort = null;
    CRServo sternStarboard = null;
    CRServo bowPort = null;
    CRServo bowStarboard = null;
    Servo plank = null;
    // DcMotor boom = null;

    Servo clawServo = null;

    double CIRCUMFERENCEOFWHEEL = 314.159; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    HardwareMap hwMap = null;

    Orientation angles;
    public IMU imu;

    Telemetry telemetry;

    /*public Robot(HardwareMap ahwMap, Telemetry tele) {
        hwMap = ahwMap;
        telemetry = tele;

        telemetry.addData("started: ", true);
        telemetry.update();

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        // Wait for the game to start (driver presses PLAY)
        imu.initialize(myIMUparameters);

        DcMotor fl = hwMap.dcMotor.get("front_left_motor");
        DcMotor fr = hwMap.dcMotor.get("front_right_motor");
        DcMotor bl = hwMap.dcMotor.get("back_left_motor");
        DcMotor br = hwMap.dcMotor.get("back_right_motor");

        DcMotor left_slide_motor = hwMap.dcMotor.get("left_slide_motor");
        DcMotor right_slide_motor = hwMap.dcMotor.get("right_slide_motor");

        TouchSensor left_touch_sensor = hwMap.touchSensor.get("lefttouch");
        TouchSensor right_touch_sensor = hwMap.touchSensor.get("righttouch");

        double ENCODERTICKS = 537.7;

        CRServo left_intake = hwMap.crservo.get("left_intake");
        CRServo right_intake = hwMap.crservo.get("right_intake");

        Servo left_claw_servo = hwMap.servo.get("left_claw_servo");
        Servo right_claw_servo = hwMap.servo.get("right_claw_servo");

        Servo left_slide_servo = hwMap.servo.get("left_slide_servo");
        Servo right_slide_servo = hwMap.servo.get("right_slide_servo");

        //Servo tilt = hardwareMap.servo.get("tilt");
        //Servo launch = hardwareMap.servo.get("launch");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        //fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        //br.setDirection(DcMotorSimple.Direction.REVERSE);


        left_slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();
    }*/

    public void init(HardwareMap ahwMap, Telemetry tele) { //pass in hardwaremap and telemetry in the code to init stuff
        hwMap = ahwMap;
        telemetry = tele;

        imu = ahwMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        // Wait for the game to start (driver presses PLAY)
        imu.initialize(myIMUparameters);

        fl = hwMap.dcMotor.get("front_left_motor");
        fr = hwMap.dcMotor.get("front_right_motor");
        bl = hwMap.dcMotor.get("back_left_motor");
        br = hwMap.dcMotor.get("back_right_motor");

        left_slide_motor = hwMap.dcMotor.get("left_slide_motor");
        right_slide_motor = hwMap.dcMotor.get("right_slide_motor");

        left_touch_sensor = hwMap.touchSensor.get("lefttouch");
        right_touch_sensor = hwMap.touchSensor.get("righttouch");

        left_intake = hwMap.crservo.get("left_intake");
        right_intake = hwMap.crservo.get("right_intake");

        left_claw_servo = hwMap.servo.get("left_claw_servo");
        right_claw_servo = hwMap.servo.get("right_claw_servo");

        left_slide_servo = hwMap.servo.get("left_slide_servo");
        right_slide_servo = hwMap.servo.get("right_slide_servo");

        //Servo tilt = hardwareMap.servo.get("tilt");
        //Servo launch = hardwareMap.servo.get("launch");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        //fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        //br.setDirection(DcMotorSimple.Direction.REVERSE);


        left_slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();
    }

    public int tickToMM(double mm){
        return (int) (mm/TICKSTOMMTRAVEL);
    }

    public void driveForwardDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(distance);
        fr.setTargetPosition(distance);
        bl.setTargetPosition(distance);
        br.setTargetPosition(distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(fl.isBusy() && bl.isBusy() && br.isBusy()) {

        }

        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void driveBackDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(-distance);
        fr.setTargetPosition(-distance);
        bl.setTargetPosition(-distance);
        br.setTargetPosition(-distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBack(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveRightDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(-distance);
        fr.setTargetPosition(-distance);
        bl.setTargetPosition(distance);
        br.setTargetPosition(distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveRight(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveLeftDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(distance);
        fr.setTargetPosition(distance);
        bl.setTargetPosition(-distance);
        br.setTargetPosition(-distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveLeft(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSlidesPos(int ticks) {

        int differenceLeft = -ticks - left_slide_motor.getCurrentPosition();
        int differenceRight = -ticks - right_slide_motor.getCurrentPosition();

        double avg = (differenceLeft + differenceRight)/2;

        if(avg >= 0) {
            left_slide_motor.setPower(2 * Math.sqrt(avg / 8100));
            right_slide_motor.setPower(2 * Math.sqrt(avg / 8100));
        }
        else {
            left_slide_motor.setPower(-2 * Math.sqrt(-avg / 8100));
            right_slide_motor.setPower(-2 * Math.sqrt(-avg / 8100));
        }
    }

    public void driveForward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void driveBack(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void driveRight(double power){
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void driveLeft(double power){
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void stopDriving() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void rotate(double wantedAngle){
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = 0;
        target = wantedAngle;


        double angle = 0;

        int totalTime = 0;

        double error = 90, P, I, D, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;
        double kp = .017;
        double ki =  0;
        double kd = .02;
        while(Math.abs(error) > 1){
            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //angle = angles.firstAngle;

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - angle;
            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            fl.setPower(correction);
            fr.setPower(-correction);
            bl.setPower(correction);
            br.setPower(-correction);

            //System.out.println(P + " " + I + " " + D);
            System.out.println(error);
            //System.out.println(angles);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("correction:", correction);
            telemetry.addData("top Left Power", fl.getPower());
            telemetry.addData("top Right Power", fr.getPower());
            telemetry.addData("bottom Left Power", bl.getPower());
            telemetry.addData("bottom Right Power", br.getPower());
            telemetry.addData("Angle", angle);
            telemetry.update();

            lastError = error;
            lastTime = t;
            totalTime += t;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("totalTime: ", totalTime * 10E-9);
        telemetry.update();

        System.out.println("angle: " + angle);
        System.out.println("totalTime: " + (totalTime * 10E-9));

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void openClaw(){
        clawServo.setPosition(.1);
    }

    public void closeClaw(){
        clawServo.setPosition(.3);
    }
}