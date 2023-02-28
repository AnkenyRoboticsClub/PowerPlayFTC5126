// IDENTIFIERS_USED=armdistanceAsDistanceSensor,color1AsColorSensor,color2AsColorSensor,controldistanceAsDistanceSensor,RedAsLED

var speed;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    RedAsLED.enableLed(true);
    while (linearOpMode.opModeIsActive()) {
      telemetry.addNumericData('arm distance sensor output, cm', armdistanceAsDistanceSensor.getDistance("CM"));
      telemetry.addNumericData('control hub distance sensor output, cm', controldistanceAsDistanceSensor.getDistance("CM"));
      telemetry.addNumericData('color sensor output 1, blue', color1AsColorSensor.getBlue());
      telemetry.addNumericData('color sensor output 1, red', color1AsColorSensor.getRed());
      telemetry.addNumericData('color sensor output 2, blue', color2AsColorSensor.getBlue());
      telemetry.addNumericData('color sensor output 2, red', color2AsColorSensor.getRed());
      telemetry.update();
    }
  }
}
