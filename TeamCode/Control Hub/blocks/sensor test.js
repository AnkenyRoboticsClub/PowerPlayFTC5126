// IDENTIFIERS_USED=armdistanceAsDistanceSensor,color1AsColorSensor,color2AsColorSensor,controldistanceAsDistanceSensor,Motor0AsDcMotor,Motor1AsDcMotor,Motor2AsDcMotor,Motor3AsDcMotor

var speed, proportion;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  Motor0AsDcMotor.setDirection("REVERSE");
  Motor1AsDcMotor.setDirection("REVERSE");
  Motor0AsDcMotor.setZeroPowerBehavior("BRAKE");
  Motor1AsDcMotor.setZeroPowerBehavior("BRAKE");
  Motor2AsDcMotor.setZeroPowerBehavior("BRAKE");
  Motor3AsDcMotor.setZeroPowerBehavior("BRAKE");
  while (!linearOpMode.opModeIsActive()) {
    telemetry.addNumericData('arm distance sensor output, cm', armdistanceAsDistanceSensor.getDistance("CM"));
    telemetry.addNumericData('control hub distance sensor output, cm', controldistanceAsDistanceSensor.getDistance("CM"));
    telemetry.addNumericData('color sensor output 1, blue', color1AsColorSensor.getBlue());
    telemetry.addNumericData('color sensor output 1, red', color1AsColorSensor.getRed());
    telemetry.addNumericData('color sensor output 2, blue', color2AsColorSensor.getBlue());
    telemetry.addNumericData('color sensor output 2, red', color2AsColorSensor.getRed());
    telemetry.update();
  }
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      telemetry.addNumericData('arm distance sensor output, cm', armdistanceAsDistanceSensor.getDistance("CM"));
      telemetry.addNumericData('control hub distance sensor output, cm', controldistanceAsDistanceSensor.getDistance("CM"));
      telemetry.addNumericData('color sensor output 1, blue', color1AsColorSensor.getBlue());
      telemetry.addNumericData('color sensor output 1, red', color1AsColorSensor.getRed());
      telemetry.addNumericData('color sensor output 2, blue', color2AsColorSensor.getBlue());
      telemetry.addNumericData('color sensor output 2, red', color2AsColorSensor.getRed());
      telemetry.update();
      speed = color1AsColorSensor.getBlue() - color2AsColorSensor.getBlue();
      proportion = 0.0007;
      if (100 > Math.abs(speed) && 200 < color1AsColorSensor.getBlue() && 200 < color2AsColorSensor.getBlue() && 400 > color1AsColorSensor.getBlue() && 400 > color2AsColorSensor.getBlue()) {
        speed = 0;
      } else if (100 > Math.abs(speed) && (180 < color1AsColorSensor.getBlue() || 180 < color2AsColorSensor.getBlue())) {
        proportion = 1;
        speed = 0.3 * (speed / Math.abs(speed));
      } else if (false) {
      } else {
      }
      Motor0AsDcMotor.setPower(speed * proportion * 1);
      Motor1AsDcMotor.setPower(speed * proportion * -1);
      Motor2AsDcMotor.setPower(speed * proportion * 1);
      Motor3AsDcMotor.setPower(speed * proportion * -1);
    }
  }
}
