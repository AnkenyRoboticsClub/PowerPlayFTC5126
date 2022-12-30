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

public class AutoV5 extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    // private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/model_20221205_155319.tflite";

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
     
    private DcMotor motorSlide;
    private CRServo right;
    private CRServo left; 
    
    private TFObjectDetector tfod;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    
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
    
    private void placeCone(){
        if (state == 0) {
            claw(-1);
            driveVertical(-170,200);//200
            pointDirection(0);
        } else if (state == 1) {
            driveHorizontal(400,200);//200
            pointDirection(0);
        } else if (state == 2) {
            driveVertical(65,200);//200
            pointDirection(0);
        } else if (state == 3) {
            claw(0.15);
            arm(200,2000);
            driveVertical(-40,200);
            pointDirection(0);
        } else if (state == 4) {
            driveHorizontal(-420,700);//400
            pointDirection(0);
        } else if (state == 5) {
            driveVertical(-2120,1000);
            pointDirection(0);
        } else if (state == 6) {
            arm(4400,2000);
            sleep(1000);
            driveHorizontal(610,600);
            sleep(500);
            pointDirection(0);
        } else if (state == 7) {
            driveVertical(95,300);
            armGravity();
            sleep(2000);
            claw(-0.1);
            pointDirection(0);
        } else if (state == 8) {
            driveVertical(-170,300);
            arm(0,3000);
            claw(-1);
            pointDirection(0);
            state++;
        } else if (state == 10) {
            endState = state;
        }
    }
    
    private void pointDirection(double direction) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
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
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            RobotLog.ii("DbgLog", "Turn: RAD=" +imu.getAngularOrientation().firstAngle +" DEG="+ Math.toDegrees(imu.getAngularOrientation().firstAngle)+" POWER="+power);
            //telemetry.addData("IMU RAD", imu.getAngularOrientation().firstAngle);
            //telemetry.addData("IMU DEGREES", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            //telemetry.addData("Power", power);
            
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
        state++;
        
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
        state++;
        
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
            tfod.setZoom(2.75, 9.0/9.0);
        }
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //Define Variables
        int tick = 0;
        int tickSS = 30;
        String imageFound = "nothing";
        boolean gotImage = false;
        //OpMode
        if (opModeIsActive()) {
            //timer.reset();
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
                        imageFound = "D3";
                    }
                }
                if ((gotImage == true)) {
                    placeCone();
                    if ((imageFound == "D1") || (imageFound == "Hawk")){
                        telemetry.addLine("1 Bolt Confirmed");
                        
                    }
                    else if ((imageFound == "D2") || (imageFound == "Hammer")) {
                        telemetry.addLine("2 Bulb Confirmed");
                        
                    }
                    else if ((imageFound == "D3") || (imageFound == "Gear")) {
                        telemetry.addLine("3 Panel Confirmed");
                        if (state == endState) {
                            driveHorizontal(-720, 2000);
                            pointDirection(0);
                            driveHorizontal(-1000, 1000);
                            pointDirection(0);
                            driveVertical(100,2000);
                            pointDirection(0);
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
