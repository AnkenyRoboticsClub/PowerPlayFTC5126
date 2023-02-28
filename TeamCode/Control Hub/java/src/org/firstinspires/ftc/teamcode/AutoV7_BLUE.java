/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Collections;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous //(name = "Concept: TensorFlow Object Detection", group = "Concept")
@Disabled
public class AutoV7_BLUE extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    // private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/model_20230114_190010.tflite";

    private static final String[] LABELS = {
        //"1 Bolt",
        //"2 Bulb",
        //"3 Panel"
        "D1",
        "D2",
        "D3",
        "Gear",
        "Hammer",
        "Hawk"
    };


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQua3MH/////AAABmUDuf1W5kURhh++lzwhKbA8KwXVHwllLTKXUN0VKW06wxewTDW2woPvacthP/3DRiCW0pizjqW2hra5ef6mIsaM/StC16g/RWDjjInNec3wkbJULFFwwAXj/bfPyFX2TCz0PrdYeCFta/k3+zRuOzZllHri8TafTAwpvb6NwhhOpq4kNyWdlY/Ruoajpn7w69NPoZs73/Wia0AdwOk5vDKdmuEnSn3hJ98zQnHhdHyxQ1i7ytXf5wdW1Bgn7Wdy1cgZ2sMIKEkQqonUf9jsO6A8vrZClIqXdjCyztTk+TKWKuOjn2wTLufTmyLFkLlwpb8pIieANG2cEXiZQkILhxrdv/FNP388w9hvli787XZjJ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
     
    private DistanceSensor armdistance;
    private DistanceSensor controldistance;
    private ColorSensor color1;
    private ColorSensor color2;
     
    private DcMotor motorSlide;
    private CRServo right;
    private CRServo left; 
    
    private TFObjectDetector tfod;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    
    private int state = 0;
    
    private  BNO055IMU imu;
    
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2*Math.PI;
            
        }
        while (radians < -Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }
    
    double integralSum = 0;
    double lastError = 0;
    //Ku is 3
    double Kp = 1.2;
    double Ki = 0;
    double Kd = 0;
    int endState = 10000;
    
    
    
    public double PIDControl (double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        
        timer.reset();
        
        double output = (error*Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
        
    }
    
    private void armDistance(int target){
        double difference = target - armdistance.getDistance(DistanceUnit.CM);
        double proportion = 0.011;
        //motorFrontLeft = hardwareMap.dcMotor.get("Motor3");
        //motorBackLeft = hardwareMap.dcMotor.get("Motor2");
        //motorFrontRight = hardwareMap.dcMotor.get("Motor0");
        //motorBackRight = hardwareMap.dcMotor.get("Motor1");
        motorFrontRight.setPower(difference * proportion * -1);
        motorBackRight.setPower(difference * proportion * -1);
        motorBackLeft.setPower(difference * proportion * -1);
        motorFrontLeft.setPower(difference * proportion * -1);
    }
    
    private void blueLine(){
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("line start");
        telemetry.update();
        double b1_bias = color1.blue()-325;
        double b2_bias = color2.blue()-325;
        double speed = b1_bias - b2_bias;
        double proportion = 0.004;
        while ((Math.abs(speed) > 20) && opModeIsActive()){
            b1_bias = color1.blue()-325;
            b2_bias = color2.blue()-325;
            speed = b1_bias - b2_bias;
            proportion = 0.004;
            //motorFrontRight.setPower(speed * proportion * 1);
            //motorBackRight.setPower(speed * proportion * -1);
            //motorBackLeft.setPower(speed * proportion * 1);
            //motorFrontLeft.setPower(speed * proportion * -1);
            telemetry.addLine("line going");
            telemetry.addData("Speed", speed);
            telemetry.addData("Power", 1/(speed*proportion));
            telemetry.update();
        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        state++;
        telemetry.addLine("line end");
        telemetry.update();
    }

    private void driveArmSideDistance(int distance){
        double diff = (distance - armdistance.getDistance(DistanceUnit.CM));

        telemetry.addData("diff=", diff);
        telemetry.update();
        driveVertical((int)(diff*-20),600);

    }
    
    private void driveToArmSide(int distance){
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && (Math.abs(distance - armdistance.getDistance(DistanceUnit.CM)) >3)){
            double dist = (distance - armdistance.getDistance(DistanceUnit.CM));
            if (Math.abs(dist)> 25){
                dist = 25*(dist/Math.abs(dist));
            }
            double power = -dist*0.025;
            //telemetry.addData("power=", power);
            //telemetry.update();
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontLeft.setPower(power);
        }
    }
    
    private boolean findBlueLine(){
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //telemetry.addLine("line start");
        //telemetry.update();
        timeout.reset();
        
        double COLOR_LIMIT = 190;
        double power = 0;
        boolean lineFound = false;
        while (opModeIsActive()){
            if ((color2.blue() >= COLOR_LIMIT) && (color1.blue() >= COLOR_LIMIT)){
                if (color2.blue() > color1.blue()){
                    // drive right slowly
                    power = 0.2; // min power to move motors right
                    //lineFound = true;
                    //break;
                } else {
                    // drive left slowly
                    power = 0.2; // min power to move motors left
                    //lineFound = true;
                    //break;
                }
            } else if ((color2.blue() > COLOR_LIMIT) && (color1.blue() < COLOR_LIMIT)){
                // drive right to line
                power = 0.2;
            } else if ((color2.blue() < COLOR_LIMIT) && (color1.blue() > COLOR_LIMIT)){
                // drive left to line
                power = -0.2;
            } else {
                power = 0;
            }
            
            motorFrontRight.setPower(power * -1);
            motorBackRight.setPower(power * 1);
            motorBackLeft.setPower(power * -1);
            motorFrontLeft.setPower(power * 1);
            //telemetry.addData("color1.blue()=",color1.blue());
            //telemetry.addData("color2.blue()",color2.blue());
            //telemetry.addData("power=", power);
            //telemetry.update();
            if ((color2.blue() < COLOR_LIMIT) && (color1.blue() < COLOR_LIMIT)){
                // Too far from line to detect
                lineFound = false;
                break;
            }else if ((Math.abs(color2.blue() - color1.blue()) < 50) &&
                !((color2.blue() < COLOR_LIMIT) && (color1.blue() < COLOR_LIMIT))){
                lineFound = true;
                break;
            }
            if (timeout.seconds()>3){
                break;
            }
        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        //state++;
        //telemetry.addLine("line end");
        //telemetry.update();
        return lineFound;
    }
    
    private void autoCycle(){
        endState = 9;
        if (state == 0) {
            //driveArmSideDistance(35);
            //driveToArmSide(85);
            claw(0.15);
            sleep(300);
            arm(1800,2000);
            //sleep(300);
            driveHorizontal(100,500);
            state++;
        } else if (state == 1) {
            pointDirection(0);
            driveHorizontal(2025,1000);
            state++;
        } else if (state == 2) {
            pointDirection(0);
            //driveToArmSide(85);
            claw(-1);
            sleep(50);
            pointDirection(0);
            driveHorizontal(800,1000);
            pointDirection(0);
            state++;
        } else if (state == 3) {
            driveHorizontal(-160,1000);
            state++;
        } else if (state == 4) {
            pointDirection(0);
            //arm(670,2000);
            driveVertical(800,1000);
            pointDirection(0);
            findBlueLine();
            pointDirection(0);
            state++;
        } else if (state == 5) {
            //arm(2500,2000);
            //sleep(300);
            driveArmSideDistance(33);
            arm(670,2000);
            sleep(700);
            state++;
        } else if (state == 6) {
            claw(0.15);
            sleep(100);
            arm(2500,2000);
            sleep(500);
            driveArmSideDistance(40);
            //pointDirection(50);
            //pointDirection(90);
            arm(2000,500);
            pointDirection(121);
            state++;
        } else if (state == 7) {
            //arm(1000,500);
            driveVertical(75,500);
            //sleep(300);
            claw(-1);
            sleep(50);
            driveVertical(-75,500);
            state++;
        } else if (state == 8) {
            pointDirection(90);
            pointDirection(90);
            driveVertical(-75,500);
            pointDirection(90);
            arm(0,2000);
            state++;
        } else if (state == 9) {
            
        }
    }
    
    private void pointDirection(double direction) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timeout.reset();
        double power = PIDControl(Math.toRadians(direction), imu.getAngularOrientation().firstAngle);
        telemetry.addLine("turning");
        telemetry.update();
        while (((Math.abs(Math.toRadians(direction)-imu.getAngularOrientation().firstAngle) > Math.toRadians(1))|| (Math.abs(power) >= 0.1)) && opModeIsActive()) {
            power = PIDControl(Math.toRadians(direction), imu.getAngularOrientation().firstAngle);
            if ((power < 0.1) && (power > 0.001)) {
                power+= 0.08;
            } else if ((power > -0.1) && (power < -0.001)) {
                power+= -0.08;
            }
            if (power > 0.5) {
                power=0.5;
            } else if (power<-0.5) {
                power=-0.5;
            }
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            RobotLog.ii("DbgLog", "Turn: RAD=" +imu.getAngularOrientation().firstAngle +" DEG="+ Math.toDegrees(imu.getAngularOrientation().firstAngle)+" POWER="+power);
            //telemetry.addData("IMU RAD", imu.getAngularOrientation().firstAngle);
            //telemetry.addData("IMU DEGREES", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            //telemetry.addData("Power", power);
            if (timeout.seconds()> 2) {
                break;
            }
            
        } 
        
        telemetry.addLine("rotation complete");
        //telemetry.addData("IMU RAD", imu.getAngularOrientation().firstAngle);
        //telemetry.addData("IMU DEGREES", Math.toDegrees(imu.getAngularOrientation().firstAngle));
        //telemetry.addData("Power", power);
        telemetry.update();
            
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        
        //Math.toRadians(direction)-imu.getAngularOrientation().firstAngle
        RobotLog.ii("DbgLog", "IMU DEG=" +Math.toDegrees(imu.getAngularOrientation().firstAngle)) ;
        RobotLog.ii("DbgLog", "IMU DEG ERROR=" +Math.toDegrees(Math.toRadians(direction)-imu.getAngularOrientation().firstAngle)) ;
        //state++;
    }
    
    private void driveVertical(int position, int velocity) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //
        motorFrontLeft.setTargetPosition(position);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorFrontLeft).setTargetPositionTolerance(10);
        ((DcMotorEx) motorFrontLeft).setVelocity(velocity);
        //
        motorBackLeft.setTargetPosition(position);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorBackLeft).setTargetPositionTolerance(10);
        ((DcMotorEx) motorBackLeft).setVelocity(velocity);
        //
        motorFrontRight.setTargetPosition(position);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorFrontRight).setTargetPositionTolerance(10);
        ((DcMotorEx) motorFrontRight).setVelocity(velocity);
        //
        motorBackRight.setTargetPosition(position);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorBackRight).setTargetPositionTolerance(10);
        ((DcMotorEx) motorBackRight).setVelocity(velocity);
        int fL = 0;
        int bR = 0;
        int fR = 0;
        int bL = 0;
        
        telemetry.addLine("driving");
        telemetry.update();
        while ((fL != position) && (bR != position) && (bL != -position) && (fR != -position) && opModeIsActive()) { 
            fL = (int)Math.round(motorFrontLeft.getCurrentPosition());
            bL = (int)Math.round(motorBackLeft.getCurrentPosition());
            fR = (int)Math.round(motorFrontRight.getCurrentPosition());
            bR = (int)Math.round(motorBackRight.getCurrentPosition());
            
            //telemetry.addLine("driving");
            //telemetry.addData("fL", motorFrontLeft.getCurrentPosition());
            //telemetry.addData("bL", motorBackLeft.getCurrentPosition());
            //telemetry.addData("fR", motorFrontRight.getCurrentPosition());
            //telemetry.addData("bR", motorBackRight.getCurrentPosition());
            //telemetry.update();
        }
        
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addLine("done driving");
        telemetry.update();
        //state++;
        
    }
    
    private void driveHorizontal(int position, int velocity) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //
        motorFrontLeft.setTargetPosition(position);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorFrontLeft).setTargetPositionTolerance(10);
        ((DcMotorEx) motorFrontLeft).setVelocity(velocity);
        //
        motorBackLeft.setTargetPosition(-position);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorBackLeft).setTargetPositionTolerance(10);
        ((DcMotorEx) motorBackLeft).setVelocity(velocity);
        //
        motorFrontRight.setTargetPosition(-position);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorFrontRight).setTargetPositionTolerance(10);
        ((DcMotorEx) motorFrontRight).setVelocity(velocity);
        //
        motorBackRight.setTargetPosition(position);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorBackRight).setTargetPositionTolerance(10);
        ((DcMotorEx) motorBackRight).setVelocity(velocity);
        int fL = 0;
        int bR = 0;
        int fR = 0;
        int bL = 0;
        telemetry.addLine("driving");
        telemetry.update();
        while ((fL != position) && (bR != position) && (bL != -position) && (fR != -position) && opModeIsActive()) { 
            fL = (int)Math.round(motorFrontLeft.getCurrentPosition());
            bL = (int)Math.round(motorBackLeft.getCurrentPosition());
            fR = (int)Math.round(motorFrontRight.getCurrentPosition());
            bR = (int)Math.round(motorBackRight.getCurrentPosition());
            
            //telemetry.addLine("driving");
            //telemetry.addData("fL", motorFrontLeft.getCurrentPosition());
            //telemetry.addData("bL", motorBackLeft.getCurrentPosition());
            //telemetry.addData("fR", motorFrontRight.getCurrentPosition());
            //telemetry.addData("bR", motorBackRight.getCurrentPosition());
            //telemetry.update();
        }
        
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addLine("done driving");
        telemetry.update();
        //state++;
        
    }
    
    private void claw(double open) {
        left.setPower(open);
        right.setPower(-open);
        sleep(1000);
    }
    
    private void arm(int height, int velocity) {
        motorSlide.setTargetPosition(-height);
        motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorSlide).setTargetPositionTolerance(10);
        ((DcMotorEx) motorSlide).setVelocity(velocity);
        //sleep(1000);
    }
    
    private void armGravity() {
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlide.setPower(0);
    }
    
    @Override
    public void runOpMode() {
        
        armdistance = hardwareMap.get(DistanceSensor.class, "arm distance");
        controldistance = hardwareMap.get(DistanceSensor.class, "control distance");
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        
        motorFrontLeft = hardwareMap.dcMotor.get("Motor3");
        motorBackLeft = hardwareMap.dcMotor.get("Motor2");
        motorFrontRight = hardwareMap.dcMotor.get("Motor0");
        motorBackRight = hardwareMap.dcMotor.get("Motor1");
        
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        right = hardwareMap.get(CRServo.class, "right");
        left = hardwareMap.get(CRServo.class, "left");
        
        motorSlide = hardwareMap.get(DcMotor.class, "MotorSlide");
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        //imu.initialize(parameters);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0/9.0);
            //tfod.setClippingMargins(0,100 ,400 ,100 );
            //tfod.setZoom(1.5, 16.0/9.0);
            
            
        }
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //Define Variables
        int tick = 0;
        int tickSS = 30;
        String imageFound = "nothing";
        String imageFound2 = "nothing";
        boolean gotImage = false;
        //OpMode
        if (opModeIsActive()) {
            timer.reset();
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() > 0) {
                            tick = tick + 1;
                        }
                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                            imageFound = recognition.getLabel();
                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                    }
                }
                //Run Actions
                if (imageFound != "nothing") {
                    //tick = tick + 1;
                }
                if ((gotImage == false) && (timer.seconds()>2)) {
                    //timer.reset();
                    gotImage = true;
                    if (imageFound == "nothing") {
                        imageFound = "D2";
                    }
                    imageFound2 = imageFound;
                }
                if ((gotImage == true)) {
                    autoCycle();
                    if ((imageFound2 == "D1") || (imageFound2 == "Hawk")){
                        telemetry.addLine("1 Bolt Confirmed");
                        if (state == endState) {
                            pointDirection(90);
                            state++;
                        }
                    }
                    else if ((imageFound2 == "D2") || (imageFound2 == "Hammer")) {
                        telemetry.addLine("2 Bulb Confirmed");
                        if (state == endState) {
                            driveHorizontal(-1000,1000);
                            pointDirection(90);
                            state++;
                        }
                    }
                    else if ((imageFound2 == "D3") || (imageFound2 == "Gear")) {
                        telemetry.addLine("3 Panel Confirmed");
                        if (state == endState) {
                            driveHorizontal(-2000,1000);
                            pointDirection(90);
                            state++;
                        }
                    }
                }
                //telemetry.addData("Timer", timer.seconds());
                //telemetry.update();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
