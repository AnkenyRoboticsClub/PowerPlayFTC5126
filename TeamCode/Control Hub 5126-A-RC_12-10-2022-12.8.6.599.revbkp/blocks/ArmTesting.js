// IDENTIFIERS_USED=gamepad1,Motor0AsDcMotor,MotorSlideAsDcMotor

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  Motor0AsDcMotor.setTargetPosition(0);
  MotorSlideAsDcMotor.setMode("RUN_USING_ENCODER");
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      MotorSlideAsDcMotor.setPower(gamepad1.getRightStickY() * 1);
      if (Motor0AsDcMotor.getCurrentPosition() < 0) {
        Motor0AsDcMotor.setTargetPosition(0);
      }
      if (Motor0AsDcMotor.getCurrentPosition() > 1800) {
        Motor0AsDcMotor.setTargetPosition(1800);
      }
      telemetry.update();
    }
  }
}
