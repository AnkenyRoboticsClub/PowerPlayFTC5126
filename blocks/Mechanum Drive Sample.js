// IDENTIFIERS_USED=frontleftAsDcMotor,frontrightAsDcMotor,gamepad1,rearleftAsDcMotor,rearrightAsDcMotor

var y, x, rx, denominator;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    frontrightAsDcMotor.setDirection("REVERSE");
    rearrightAsDcMotor.setDirection("REVERSE");
    while (linearOpMode.opModeIsActive()) {
      y = -(gamepad1.getLeftStickY() * 0.5);
      x = gamepad1.getRightStickX() * 0.5;
      rx = gamepad1.getLeftStickX() * -0.6;
      denominator = Math.max.apply(null, [[Math.abs(y), Math.abs(x), Math.abs(rx)].reduce(function(x, y) {return x + y;}), 1]);
      frontleftAsDcMotor.setPower((y + x + rx) / denominator);
      rearleftAsDcMotor.setPower(((y - x) + rx) / denominator);
      frontrightAsDcMotor.setPower(((y - x) - rx) / denominator);
      rearrightAsDcMotor.setPower(((y + x) - rx) / denominator);
      telemetry.update();
    }
  }
}
