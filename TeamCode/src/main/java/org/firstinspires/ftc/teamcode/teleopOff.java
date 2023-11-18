package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



//TELEOP WITH 1 CONTROLLER
@TeleOp(name = "Tele w/ 1")
public class teleopOff extends LinearOpMode {
    public void runOpMode() {

        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        //DcMotor lift1 = hardwareMap.dcMotor.get("liftMotor1");
        //DcMotor lift2 = hardwareMap.dcMotor.get("liftMotor2");
        DcMotor fourBar = hardwareMap.dcMotor.get("4bar_motor");

        double ENCODERTICKS = 537.7;

        CRServo back_left_servo = hardwareMap.crservo.get("back_left_servo");
        CRServo back_right_servo = hardwareMap.crservo.get("back_right_servo");
        CRServo front_left_servo = hardwareMap.crservo.get("front_left_servo");
        CRServo front_right_servo = hardwareMap.crservo.get("front_right_servo");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        //br.setDirection(DcMotorSimple.Direction.REVERSE);

        /* lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
        fourBar.setTargetPosition(0);
        // fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Servo outtake = hardwareMap.servo.get("4bar_servo");

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double driveSpeed = 0.4;
        final int liftHome = 0;
        double robotAngle = 0;
        boolean fieldCentric = false;
        double finalAngle = 0;
        double liftEncoder;
        boolean aButton = true;
        boolean yButton = true;
        boolean xButton = true;
        boolean bButton = true;
        int clawPos = 0;
        //test
        double plankPos = 0;

        Orientation angles;
        IMU imu;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters); */

        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            // DRIVE SPEED
            if (gamepad1.a && aButton) {
                aButton = false;
                if (driveSpeed == 0.7) { //if the current increment is 1, it'll switch to 0.5
                    driveSpeed = 0.4;
                } else { //if the current increment is not 1, it'll switch to 1
                    driveSpeed = 0.7;
                }
            }
            if (!gamepad1.a && !aButton) {
                aButton = true;
            }

            // FIELD CENTRIC
            if(gamepad1.y && yButton) {
                yButton = false;
                if(fieldCentric){
                    fieldCentric = false;
                }
                else{
                    fieldCentric = true;
                }
                //sleep(1000);
            }
            if (!gamepad1.y && !yButton) {
                yButton = true;
            }


            // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            // robotAngle = angles.firstAngle;

            //DRIVE CONTROL
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y); //finds hypotenuse (power of each motor)
            double gpAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //finds angle of robot subtracted by pi/4 bc
            //it "shifts" the powers to each motor CW
            double rightX = (-gamepad1.right_stick_x) * .8; //for rotating w/ right stick

            if(fieldCentric){
                finalAngle = gpAngle - robotAngle;
            }
            else{
                finalAngle = gpAngle;
            }

            final double v1 = driveSpeed * (r * Math.cos(finalAngle) + rightX);
            final double v2 = driveSpeed * (r * Math.sin(finalAngle) - rightX);
            final double v3 = driveSpeed * (r * Math.sin(finalAngle) + rightX);
            final double v4 = driveSpeed * (r * Math.cos(finalAngle) - rightX);
            /*
                r is multiplier for power
                math.cos is used for fl and br bc fl&br are used to go diagonal top right, if you want to go faster to the right apply more power to
                those motors so closer joystick is to x axis faster robot go to that direction
                math.sin is used for same reason as ^ but to go faster forward/backwards
             */

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);


            // INTAKE SERVOS
            if(gamepad1.right_bumper) {
                front_left_servo.setPower(-1);
                front_right_servo.setPower(1);
                back_left_servo.setPower(-1);
                back_right_servo.setPower(1);
            }
            if(gamepad1.left_bumper) {
                front_left_servo.setPower(1);
                front_right_servo.setPower(-1);
                back_left_servo.setPower(1);
                back_right_servo.setPower(-1);
            }
            if(!gamepad1.right_bumper && !gamepad1.left_bumper) {
                front_left_servo.setPower(0);
                front_right_servo.setPower(0);
                back_left_servo.setPower(0);
                back_right_servo.setPower(0);
            }

            if(gamepad1.right_trigger > 0) {
                fourBar.setPower(gamepad1.right_trigger/1.5);
            }
            if(gamepad1.left_trigger > 0) {
                fourBar.setPower(-gamepad1.left_trigger/1.5);
            }
            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
                fourBar.setPower(0);
            }

            if(gamepad1.dpad_down) {
                outtake.setPosition(0);
                //fourBar.setTargetPosition(50);
                if(fourBar.getCurrentPosition() >= 50) {
                    //fourBar.setPower(-0.05);
                }
                else {
                    fourBar.setPower(0);
                }
            }
            if(gamepad1.dpad_up) {
                outtake.setPosition(0.5);
                //fourBar.setTargetPosition(20);
                if(fourBar.getCurrentPosition() <= 20) {
                    //fourBar.setPower(-0.05);
                }
                else {
                    fourBar.setPower(0);
                }
            }
            if(gamepad1.dpad_right) {
                outtake.setPosition(1);
            }


            telemetry.addData("Drive Speed Multiplier", driveSpeed);
            telemetry.addData("Field-Centric", fieldCentric);
            //telemetry.addData("lift1 enc ticks", lift1.getCurrentPosition());
            //telemetry.addData("lift2 enc ticks", lift2.getCurrentPosition());
            //telemetry.addData("Servo Pos", clawPos);
            //telemetry.addData("lift motor power", gamepad1.right_trigger);
            // telemetry.addData("angle", angles.firstAngle);
            //telemetry.addData("bucketServoPos", plankPos);
            telemetry.addData("4Bar Encoder: ", fourBar.getCurrentPosition());
            telemetry.addData("4Bar dest. ", fourBar.getTargetPosition());
            telemetry.addData("4Bar power: ", fourBar.getPower());
            telemetry.addData("outtake encoder: ", outtake.getPosition());
            telemetry.update();
        }
    }
}
