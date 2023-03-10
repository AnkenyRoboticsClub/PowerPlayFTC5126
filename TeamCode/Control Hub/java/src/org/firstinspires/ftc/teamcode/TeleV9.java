package org.firstinspires.ftc.teamcode;
//import com.robotcore.DigitalChannel;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.*;
//import java.util.List;
//import java.util.Scanner;

@TeleOp
public class TeleV9 extends LinearOpMode {
    private DcMotor MotorSlide;
    private CRServo right;
    private CRServo left;
    private LED Red;
    private LED Green;
    //private DistanceSensor armDistance;
    private DistanceSensor controlDistance;
    //Scanner scan=new Scanner(System.in);
    //String s=scan.nextLine();
    /*
    double live_gyro_value = 0;
    double gyro_offset = 0;
    */
    /*
    double getHeading() {
        // read the gyro
        live_gyro_value = imu.getAngularOrientation().firstAngle; //.... code to read gyro here.
        return (live_gyro_value - gyro_offset);
    }

    void reset_gyro() {
        gyro_offset = live_gyro_value;
    }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        controlDistance = hardwareMap.get(DistanceSensor.class, "control distance");
        
        // Speed Variable
        double speed = 0.5;
        
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Motor3");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Motor2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Motor0");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Motor1");
        
        right = hardwareMap.get(CRServo.class, "right");
        left = hardwareMap.get(CRServo.class, "left");
        //right.setPower(-1);
        //left.setPower(1);
        
        // Zero power mode
        //motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeverRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        
        //ARM INIT
        int encoderPosition;
        int lastEncoderPosition;
        int encoderReal;
        int encoderMin;
        int encoderMax;
        int pole1;
        int pole2;
        int pole3;
        int stack;
        int dist1=0;
        int dist2=0;
        int dist3=0;
        int dist4=0;

        int toggleLeftTrigger=0;
        
        
        MotorSlide = hardwareMap.get(DcMotor.class, "MotorSlide");
        MotorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderPosition = 0;
        encoderMin = -4411;
        encoderMax = 0;
        pole1 = -1845;
        pole2 = -2900;
        pole3 = encoderMin+20;
        stack = -670;
        //MotorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //((DcMotorEx) MotorSlide).setTargetPositionTolerance(10);
        //MotorSlide.setPower(1);
        //MotorSlide.setTargetPosition(encoderPosition);
        // Put initialization blocks here.
        Red=hardwareMap.get(LED.class,"Red");
        Green=hardwareMap.get(LED.class,"Green");

        double live_gyro_value = 0;
        double gyro_offset = 0;
        //double triggerToggle = 0;

        waitForStart();
        //Red.setMode(DigitalChannel.Mode.OUTPUT);
        //Green.setMode(DigitalChannel.Mode.OUTPUT);
        if(isStopRequested()) 
        {
            return;
        }
        
        while (opModeIsActive()) {
            
            //put pole sensor here
            if(controlDistance.getDistance(DistanceUnit.CM)>18)
            {
                
                Red.enable(true);
                Green.enable(false);
            }
            else if(controlDistance.getDistance(DistanceUnit.CM)<13)
            {
                Red.enable(true);
                Green.enable(true);
            }
            else
            {
                Red.enable(false);
                Green.enable(true);
            }
            
            
            //lastEncoderPosition = encoderPosition;
            encoderReal = (int)Math.round(MotorSlide.getCurrentPosition());
            
            if (encoderReal<stack) {
                left.setPower(gamepad2.right_trigger * 1 - 0.7);
                right.setPower(gamepad2.right_trigger * -1 + 0.7);
            } else {
                left.setPower(gamepad2.right_trigger * 0.5 - 0.25);
                right.setPower(gamepad2.right_trigger * -0.5 + 0.25);
            }
            
            if (Math.abs(gamepad2.right_stick_y) > 0.1){
                encoderPosition = encoderReal + (int)Math.round(gamepad2.right_stick_y) * 150;
            }
            
            
            //if (gamepad2.right_stick_y < 0) {
                //encoderPosition += gamepad2.right_stick_y * 15; //up
            //} else {
                //encoderPosition += gamepad2.right_stick_y * 10; //down
            //}
            //encoderPosition = Math.toIntExact(MotorSlide.getCurrentPosition()) + gamepad2.right_stick_y * 10;
            //if (gamepad2.right_stick_y!=Math.abs(gamepad2.right_stick_y))
            //{
                //MotorSlide.setPower(gamepad2.right_stick_y/3);
                
            //}
            //else{
            //MotorSlide.setPower(gamepad2.right_stick_y);
            //}

            if(toggleLeftTrigger==1 && gamepad2.left_trigger<0.5){
                toggleLeftTrigger=0;
                encoderMin = -4411+encoderReal-10;
                encoderMax = 0+encoderReal-10;
                pole1 = -1845+encoderReal-10;
                pole2 = -2900+encoderReal-10;
                pole3 = encoderMin+20-10;
                stack = -670+encoderReal-10;
            }
            if(gamepad2.left_trigger>0.5){
                toggleLeftTrigger=1;
            } else {
                toggleLeftTrigger=0;
                if (encoderPosition < encoderMin) {
                    encoderPosition = encoderMin;
                }
                if (encoderPosition > encoderMax) {
                    encoderPosition = encoderMax;
                }
            }
            dist1 = Math.abs(encoderReal - pole1);
            dist2 = Math.abs(encoderReal - pole2);
            dist3 = Math.abs(encoderReal - pole3);
            dist4 = Math.abs(encoderReal - stack);

            if(gamepad2.a) {
                if ((dist4 < dist1) && (dist4 < dist2) && (dist4 < dist3)) {
                    encoderPosition = stack;
                }
                if ((dist3 < dist1) && (dist3 < dist2) && (dist3 < dist4)) {
                    encoderPosition = pole3;
                }
                if ((dist2 < dist1) && (dist2 < dist4) && (dist2 < dist3)) {
                    encoderPosition = pole2;
                }
                if ((dist1 < dist4) && (dist1 < dist2) && (dist1 < dist3)) {
                    encoderPosition = pole1;
                }
            }

            if(gamepad2.x) {
                encoderPosition = pole3;
            }
            if(gamepad2.b) {
                encoderPosition = encoderMax;
            }

            MotorSlide.setTargetPosition(encoderPosition);
            MotorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) MotorSlide).setTargetPositionTolerance(10);
            ((DcMotorEx) MotorSlide).setVelocity(2000);

            /*if (lastEncoderPosition < encoderPosition) {
                //((DcMotorEx) MotorSlide).setVelocity(500);
                
                if (MotorSlide.getCurrentPosition() < -700) {
                    ((DcMotorEx) MotorSlide).setVelocity(300);
                } else {
                    ((DcMotorEx) MotorSlide).setVelocity(100);
                }
                
                //((DcMotorEx) MotorSlide).setVelocity(500);//down
                
            } else {
                ((DcMotorEx) MotorSlide).setVelocity(2000);//up
            }
            */
            
            if (encoderReal < pole2+200){
                speed = 0.4;
            } else if(gamepad1.a) {
                speed = 1;
            } else {
                speed = 0.5;
            }

            if(gamepad1.x) {
                gyro_offset = live_gyro_value;
            }

            
            double y = -gamepad1.left_stick_y;          // Up-Down
            double x =  gamepad1.left_stick_x * 1.1;    // Left-Right
            double rx = gamepad1.right_stick_x;         //Rotation

            // Read inverse IMU heading, as the IMU heading is CW positive

            live_gyro_value = -imu.getAngularOrientation().firstAngle;
            double botHeading = live_gyro_value - gyro_offset;

            //double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            
            frontLeftPower = frontLeftPower * speed;
            frontRightPower = frontRightPower * speed;
            backLeftPower = backLeftPower * speed;
            backRightPower = backRightPower * speed;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            
            telemetry.addData("Motor","%.2f, %.2f, %.2f, %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("IMU",imu.getAngularOrientation().firstAngle);
            telemetry.addData("distance to pole 1", dist1);
            telemetry.addData("distance to pole 2", dist2);
            telemetry.addData("distance to pole 3", dist3);
            telemetry.addData("distance to stack", dist4);
            telemetry.addData("target", encoderPosition);
            telemetry.addData("real", MotorSlide.getCurrentPosition());
            telemetry.addData("motor power", MotorSlide.getPower());
            telemetry.addData("Pole Distance",controlDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
            
        }
    }
}
