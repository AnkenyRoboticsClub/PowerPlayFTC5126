// IDENTIFIERS_USED=gamepad1,MotorSlideAsDcMotor

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  MotorSlideAsDcMotor.setMode("STOP_AND_RESET_ENCODER");
  MotorSlideAsDcMotor.setMode("RUN_USING_ENCODER");
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      MotorSlideAsDcMotor.setPower(gamepad1.getRightStickY());
      telemetry.addNumericData('motor power', MotorSlideAsDcMotor.getPower());
      telemetry.addNumericData('encoder', MotorSlideAsDcMotor.getCurrentPosition());
      telemetry.update();
    }
  }
}
