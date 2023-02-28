// IDENTIFIERS_USED=armdistanceAsDistanceSensor,color1AsColorSensor,color2AsColorSensor,controldistanceAsDistanceSensor,Motor0AsDcMotor,Motor1AsDcMotor,Motor2AsDcMotor,Motor3AsDcMotor

var difference, proportion;

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
    telemetry.addNumericData('front distance sensor output, cm', controldistanceAsDistanceSensor.getDistance("CM"));
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
      difference = 35 - armdistanceAsDistanceSensor.getDistance("CM");
      proportion = 0.011;
      Motor0AsDcMotor.setPower(difference * proportion * -1);
      Motor1AsDcMotor.setPower(difference * proportion * -1);
      Motor2AsDcMotor.setPower(difference * proportion * -1);
      Motor3AsDcMotor.setPower(difference * proportion * -1);
    }
  }
}
