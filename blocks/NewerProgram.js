// IDENTIFIERS_USED=gamepad1,MotorSlideAsDcMotor

var encoderPosition, encoderMin, encoderMax, pole1, pole2, pole3, dist1, dist2, dist3;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  encoderPosition = 0;
  encoderMin = -2403;
  encoderMax = 0;
  pole1 = -1150;
  pole2 = -1740;
  pole3 = encoderMin;
  MotorSlideAsDcMotor.setMode("RUN_USING_ENCODER");
  MotorSlideAsDcMotor.setMode("RUN_TO_POSITION");
  MotorSlideAsDcMotor.setTargetPositionTolerance(10);
  MotorSlideAsDcMotor.setPower(1);
  if (gamepad1.getA()) {
    MotorSlideAsDcMotor.setMode("STOP_AND_RESET_ENCODER");
  }
  MotorSlideAsDcMotor.setTargetPosition(encoderPosition);
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      encoderPosition = (typeof encoderPosition == 'number' ? encoderPosition : 0) + gamepad1.getRightStickY() * 5;
      if (encoderPosition < encoderMin) {
        encoderPosition = encoderMin;
      }
      if (encoderPosition > encoderMax) {
        encoderPosition = encoderMax;
      }
      dist1 = encoderPosition - pole1;
      dist2 = encoderPosition - pole2;
      dist3 = encoderPosition - pole3;
      MotorSlideAsDcMotor.setTargetPosition(encoderPosition);
      telemetry.addNumericData('distance to pole 1', dist1);
      telemetry.addNumericData('distance to pole 2', dist2);
      telemetry.addNumericData('distance to pole 3', dist3);
      telemetry.addNumericData('target', encoderPosition);
      telemetry.addNumericData('real', MotorSlideAsDcMotor.getCurrentPosition());
      telemetry.addNumericData('motor power', MotorSlideAsDcMotor.getPower());
      telemetry.update();
    }
  }
}
