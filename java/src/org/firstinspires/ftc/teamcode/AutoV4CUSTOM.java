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

public class AutoV4CUSTOM extends LinearOpMode {

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
    private TFObjectDetector tfod;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    @Override
    public void runOpMode() {
        
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Motor3");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Motor2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Motor0");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Motor1");
        
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
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
                    tick = tick + 1;
                }
                if ((gotImage == false) && (tick >= tickSS)) {
                    timer.reset();
                    gotImage = true;
                }
                if ((gotImage == true)||(timer.seconds()>10)) {
                    if ((imageFound == "D1") || (imageFound == "Hawk")){
                        telemetry.addLine("1 Bolt Confirmed");
                        if (timer.seconds() < 1.5) {
                          motorFrontLeft.setPower(0.5);
                           motorBackLeft.setPower(-0.5);
                           motorFrontRight.setPower(-0.5);
                          motorBackRight.setPower(0.5);
                        } 
                        if ((timer.seconds() > 1.5) && (timer.seconds() < 2.8)) {
                            motorFrontLeft.setPower(-0.4);
                            motorBackLeft.setPower(-0.4);
                            motorFrontRight.setPower(-0.4);
                            motorBackRight.setPower(-0.4);
                        }
                        if (timer.seconds() > 2.8) {
                            motorFrontLeft.setPower(0);
                            motorBackLeft.setPower(0);
                            motorFrontRight.setPower(0);
                            motorBackRight.setPower(0);
                        }
                    }
                    if ((imageFound == "D2") || (imageFound == "Hammer")) {
                        telemetry.addLine("2 Bulb Confirmed");
                        if (timer.seconds() < 3) {
                            motorFrontLeft.setPower(-0.25);
                            motorBackLeft.setPower(-0.25);
                            motorFrontRight.setPower(-0.25);
                            motorBackRight.setPower(-0.25);
                        } 
                        if (timer.seconds() > 3) {
                            motorFrontLeft.setPower(0);
                            motorBackLeft.setPower(0);
                            motorFrontRight.setPower(0);
                            motorBackRight.setPower(0);
                        }
                    }
                    if ((imageFound == "D3") || (imageFound == "Gear")||(gotImage==false)) {
                        telemetry.addLine("3 Panel Confirmed");
                        if (timer.seconds() < 1.5) {
                            motorFrontLeft.setPower(-0.5);
                            motorBackLeft.setPower(0.5);
                            motorFrontRight.setPower(0.5);
                            motorBackRight.setPower(-0.5);
                        } 
                        if ((timer.seconds() > 1.5) && (timer.seconds() < 2.8)) {
                            motorFrontLeft.setPower(-0.4);
                            motorBackLeft.setPower(-0.4);
                            motorFrontRight.setPower(-0.4);
                            motorBackRight.setPower(-0.4);
                        }
                        if (timer.seconds() > 2.8) {
                            motorFrontLeft.setPower(0);
                            motorBackLeft.setPower(0);
                            motorFrontRight.setPower(0);
                            motorBackRight.setPower(0);
                        }
                    }
                }
                telemetry.addData("Timer", timer.seconds());
                telemetry.update();
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
