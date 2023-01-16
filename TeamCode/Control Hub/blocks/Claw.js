// IDENTIFIERS_USED=gamepad1,leftAsCRServo,rightAsCRServo

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  rightAsCRServo.setPower(-1);
  leftAsCRServo.setPower(1);
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      leftAsCRServo.setPower(gamepad1.getRightTrigger() * 1.15 - 0.15);
      rightAsCRServo.setPower(gamepad1.getRightTrigger() * -1.15 + 0.15);
      telemetry.update();
    }
  }
}
