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


//Import Files
package org.firstinspires.ftc.teamcode;
import java.lang.*;
import com.qualcomm.robotcore.hardware.LED;
import java.util.Arrays;
import java.lang.reflect.Array;
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
 
//Set file to autonomous
@Autonomous 
@Disabled //Used disable files

//Make sure the name is correct
public class autov9RedTest extends LinearOpMode {


    //Camera Initialization
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    // private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // replace after "tflitemodels/"
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/model_20230114_190010.tflite";

    private static final String[] LABELS = {
        //labels
        "D1",//side 1
        "D2",//side 2
        "D3",//side 3
        "Gear",//side 3
        "Hammer",//side 2
        "Hawk"//side 1
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
     
    //initialize component variables
    //sensors
    private DistanceSensor armdistance;
    private DistanceSensor controldistance;
    private ColorSensor color1;
    private ColorSensor color2;
    //can't use this color sensor because it's illegal
    private LED Red;
    private LED Green;
     
    //arm motor and servos
    private DcMotor motorSlide;
    private CRServo right;
    private CRServo left; 
    
    //camera
    private TFObjectDetector tfod;
    
    //timers
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    
    //wheel motors
    private DcMotor motorFrontLeft;     //3
    private DcMotor motorBackLeft;      //2
    private DcMotor motorFrontRight;    //0
    private DcMotor motorBackRight;     //1
    
    //set the state for the state machine
    private int state = -1;
    
    //initialize IMU
    private  BNO055IMU imu;
    
    //finds optimal rotation
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2*Math.PI;
            
        }
        while (radians < -Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }
    
    //configure PID Controller for rotation
    double integralSum = 0;
    double lastError = 0;
    //Ku is 3
    double Kp = 1.2;
    double Ki = 0;
    double Kd = 0;
    
    //set endstate to a high number to prevent end from starting
    int endState = 10000;
    
    //PID control method
    public double PIDControl (double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        
        timer.reset();
        
        double output = (error*Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
        
    }
    
    //drive based on distance senosr reading from controll hub side
    private void driveArmSideDistance(double distance){
        double diff = (distance - armdistance.getDistance(DistanceUnit.CM));
        double average=0;
        
        //finds the average of the values collected by the distance sensor to increase accuracy.
        List<Double> values= Arrays.asList(0.0,0.0,0.0,0.0,0.0,0.0);
        for(int i=0;i<6;i++)
        {
            average=average+armdistance.getDistance(DistanceUnit.CM);
            sleep(100);
            values.set(i,armdistance.getDistance(DistanceUnit.CM));
            
        }
        average=average/6;
        Double[]vals=values.toArray(new Double[values.size()]);
        //gets the average
        double avgDiff=10000;
        double smallestDev=0;
        for(int i=0;i<6;i++)
        {
            if(Math.abs(vals[i]-average)<avgDiff)
            {
                avgDiff=vals[i]-average;
                smallestDev=vals[i];
            }
        }
        diff = (distance - smallestDev);
        telemetry.addData("diff=", diff);
        telemetry.update();
        //converts centimeters to encoder units
        //diff*-30 (-30 is the multiplier)
        driveVertical((int)(diff*-30),600);
    }
    private int driveToWall(){
       return (int)((armdistance.getDistance(DistanceUnit.CM)-25)*10);
    }
    
    //finds the red line on the field
    private boolean findRedLine(){
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //reset timeout
        timeout.reset();
        //color limit variable. the optimal color reading
        double COLOR_LIMIT = 150;
        double power = 0;
        boolean lineFound = false;
        while (opModeIsActive()){
            if ((color2.red() >= COLOR_LIMIT) && (color1.red() >= COLOR_LIMIT)){
                if (color2.red() > color1.red()){
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
            } else if ((color2.red() > COLOR_LIMIT) && (color1.red() < COLOR_LIMIT)){
                // drive right to line
                power = 0.2;
            } else if ((color2.red() < COLOR_LIMIT) && (color1.red() > COLOR_LIMIT)){
                // drive left to line
                power = -0.2;
            } else {
                power = 0;
            }
            
            motorFrontRight.setPower(power * -1);
            motorBackRight.setPower(power * 1);
            motorBackLeft.setPower(power * -1);
            motorFrontLeft.setPower(power * 1);
            if ((color2.red() < COLOR_LIMIT) && (color1.red() < COLOR_LIMIT)){
                // Too far from line to detect
                lineFound = false;
                break;
            }else if ((Math.abs(color2.red() - color1.red()) < 50) &&
                !((color2.red() < COLOR_LIMIT) && (color1.red() < COLOR_LIMIT))){
                lineFound = true;
                break;
            }
            if (timeout.seconds()>3){
                lineFound = false;
                break;
            }
        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        //returns the if the line was found or not
        return lineFound;
    }
    
    //set led color like in tele to show distance from pole
    //color sensor is illegal but it doesn't report if it is connected or not so just leave the code
    private double getAbsolutePoleDist()
    {
        if(controldistance.getDistance(DistanceUnit.CM)>18)
            {
                //too far
                Red.enable(true);
                Green.enable(false);
                return(controldistance.getDistance(DistanceUnit.CM)-18); 
                
            }
            else if(controldistance.getDistance(DistanceUnit.CM)<13)
            {
                //too close
                Red.enable(true);
                Green.enable(true);
                return(controldistance.getDistance(DistanceUnit.CM)+13);
            }
            else
            {
                //in range
                Red.enable(false);
                Green.enable(true);
                return (0);
            }
    
    }
    private int usePoleSensor(){
        
        if(controldistance.getDistance(DistanceUnit.CM)>18)
            {
                //too far
                Red.enable(true);
                Green.enable(false);
                return(0); 
                
            }
            else if(controldistance.getDistance(DistanceUnit.CM)<13)
            {
                //too close
                Red.enable(true);
                Green.enable(true);
                return (2);
            }
            else
            {
                //in range
                Red.enable(false);
                Green.enable(true);
                return (1);
            }
    }
    
    //points in direction specified using PID Controller and IMU.
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
            if (timeout.seconds()> 2) {
                break;
            }
            
        } 
        
        telemetry.addLine("rotation complete");
        telemetry.update();
            
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        
        RobotLog.ii("DbgLog", "IMU DEG=" +Math.toDegrees(imu.getAngularOrientation().firstAngle)) ;
        RobotLog.ii("DbgLog", "IMU DEG ERROR=" +Math.toDegrees(Math.toRadians(direction)-imu.getAngularOrientation().firstAngle)) ;
    }
    
    //drives vertically
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
        }
        
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addLine("done driving");
        telemetry.update();
        
    }
    
    //drives horizontally
    private void driveHorizontal(double position, int velocity) {//changed from double
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //
        motorFrontLeft.setTargetPosition((int)position);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorFrontLeft).setTargetPositionTolerance(10);
        ((DcMotorEx) motorFrontLeft).setVelocity(velocity);
        //
        motorBackLeft.setTargetPosition((int)-position);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorBackLeft).setTargetPositionTolerance(10);
        ((DcMotorEx) motorBackLeft).setVelocity(velocity);
        //
        motorFrontRight.setTargetPosition((int)-position);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorFrontRight).setTargetPositionTolerance(10);
        ((DcMotorEx) motorFrontRight).setVelocity(velocity);
        //
        motorBackRight.setTargetPosition((int)position);
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
        }
        
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addLine("done driving");
        telemetry.update();
    }
    
    //operates the claw
    private void claw(double open) {
        left.setPower(open);
        right.setPower(-open);
        sleep(1000);
    }
    
    //operates the arm
    private void arm(int height, int velocity) {
        motorSlide.setTargetPosition(-height);
        motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorSlide).setTargetPositionTolerance(10);
        ((DcMotorEx) motorSlide).setVelocity(velocity);
        //sleep(1000);
    }
    //causes the arm to drop using gravity
    private void armGravity() {
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlide.setPower(0);
    }
    
    //state machine to progress through the cycles
    private void autoCycle(){
        if (state== -1) {
            claw(0.15);
            sleep(300);
            state++;
        }else if (state == 0) {
            arm(4350,2000);
            state++;
        } else if (state == 1) {
            for (int i =1;i<3;i++)
            {
                int temp=1000*i;
                driveHorizontal(1662,temp);
            }
           // driveHorizontal(3325,2000);//2525 too short
            driveVertical(125,500);
            state++;
        } else if (state == 2) {
            if(getAbsolutePoleDist()>90)
            {
                int lineUp=0;
                while(lineUp<4)
                {
                    if(lineUp%2==0)
                    {
                        driveHorizontal(100,1000);
                    }
                    if(lineUp%2==1)
                    {
                        driveHorizontal(-100,1000);
                    }
                    if(getAbsolutePoleDist()<90)
                    {
                        break;
                    }
                }
            }
           int tries =0;
            while((tries<3))//this gives the robot three tries 
            //to line up with the pole
            {
                
                if(usePoleSensor()==0)
                {
                    driveVertical((int)getAbsolutePoleDist(),300);//75
               
                }   
                else if(usePoleSensor()==1)
                {
                    arm(2000,2000);
                    break; 
                }
                else if (usePoleSensor()==2)
                {
                    driveVertical((int)getAbsolutePoleDist(),300);//-100
                }
                tries++;
            }
            
            
           // arm(1200,2000);//edit me
            claw(-1);
            sleep(50);
            arm(1800,2000);
            pointDirection(0);
            driveVertical(30,500);
            driveHorizontal(-800,1000);
            pointDirection(0);
            state++;
        } else if (state == 3) {
            pointDirection(180);
            driveHorizontal(-160,1000);
            state++;
            
        } else if (state == 4) {
            pointDirection(180);
            driveVertical(750,1000);
            pointDirection(180);
            findRedLine();
            pointDirection(180);
            state++;
        } else if (state == 5) {
            //sleep(500);
           for (int i=0;i<3;i++)
           {
               driveVertical(driveToWall(),400);
           }
            //changed from driveArmSideDistance(30);
            pointDirection(180);
            arm(670,2000);
            sleep(700);
            state++;
        } else if (state == 6) {
            claw(0.15);
            sleep(100);
            arm(2500,2000);
            sleep(500);
            pointDirection(180);
            // driveArmSideDistance(36);
            driveVertical(-200,1000);
            //pointDirection(180);
            arm(2000,500);
            pointDirection(70);
            state++;
        } else if (state == 7) {
            driveVertical(100,500);
            if(getAbsolutePoleDist()>90)
            {
                int lineUp=0;
                while(lineUp<4)
                {
                    if(lineUp%2==0)
                    {
                        driveHorizontal(100,1000);
                    }
                    if(lineUp%2==1)
                    {
                        driveHorizontal(-100,1000);
                    }
                    if(getAbsolutePoleDist()<90)
                    {
                        break;
                    }
                }
            }
            int tries =0;
            while((tries<3))//this gives the robot three tries 
            //to line up with the pole
            {
                
                if(usePoleSensor()==0)
                {
                    driveVertical((int)getAbsolutePoleDist(),500);//75
               
                }   
                else if(usePoleSensor()==1)
                {
                   // break; 
                }
                else if (usePoleSensor()==2)
                {
                    driveVertical((int)getAbsolutePoleDist(),500);//-100
                }
                tries++;
            }
            arm(1100,2000);//force cone down
            claw(-1);
            sleep(50);
            arm(1800,2000);
            driveVertical(30,500);
            state++;
        } else if (state == 8) {
            pointDirection(90);
            pointDirection(90);
            driveVertical(-75,500);
            pointDirection(90);
            arm(0,2000);
            state++;
            pointDirection(90);
        } else if (state == 9) {
            endState = 9;
        }
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
        
        Red=hardwareMap.get(LED.class,"Red");
        Green=hardwareMap.get(LED.class,"Green");
        
        
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
                autoCycle();
                if ((gotImage == true)) {
                    
                    if ((imageFound2 == "D1") || (imageFound2 == "Hawk")){
                        telemetry.addLine("1 Bolt Confirmed");
                        if (state == endState) {
                            driveHorizontal(-400,1000);
                            //pointDirection(90);
                            state++;
                        }
                    }
                    else if ((imageFound2 == "D2") || (imageFound2 == "Hammer")) {
                        telemetry.addLine("2 Bulb Confirmed");
                        if (state == endState) {
                            driveHorizontal(700,1000);
                            //pointDirection(90);
                            state++;
                        }
                    }
                    else if ((imageFound2 == "D3") || (imageFound2 == "Gear")) {
                        telemetry.addLine("3 Panel Confirmed");
                        if (state == endState) {
                            driveHorizontal(2000,1000);
                            //pointDirection(90);
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
