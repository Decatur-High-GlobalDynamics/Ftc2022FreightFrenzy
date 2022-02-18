package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Class to wrap a DcMotor, particularly with telemetry data,
 * including speed information that requires information to be saved
 * by the Motor between loops
 *
 */
public class TeamServo {
    final Robot robot;
    final String name;
    final Servo servo;
    double currentPosition;
    private Double minimumSafetyPosition, maximumSafetyPosition;
    private Double minPositionSeen, maxPositionSeen;
    private boolean limitsEnabled = false;

    private Long positionChangeTime_ms; // when the last order to go to a position was issued

    public TeamServo(Robot robot, OpMode opMode, String name) {
        this(robot, name, opMode.hardwareMap.servo.get(name));
    }

    public TeamServo(Robot robot, String name, Servo servo) {
        this.robot = robot;
        this.name = name;
        this.servo = servo;
    }

    public void loop() {
        currentPosition = servo.getPosition();

        // Make note of the min and max positions ever seen for this servo
        if ( minPositionSeen == null || minPositionSeen > currentPosition )
            minPositionSeen = currentPosition;
        if ( maxPositionSeen == null || maxPositionSeen < currentPosition )
            maxPositionSeen = currentPosition;
    }

    public String getServoTelemetryString() {
        return String.format("pos %.2f | range %.2f - %.2f", servo.getPosition(), minPositionSeen, maxPositionSeen);
    }

    public double getPositionAboveMinimumSeen() {
        if (minPositionSeen == null)
            return 0;
        return currentPosition - minPositionSeen;
    }

    public double getPositionBelowMaximumSeen() {
        if (maxPositionSeen == null)
            return 0;
        return maxPositionSeen - currentPosition;
    }

    public ServoController getController() {
        return servo.getController();
    }

    public int getPortNumber() {
        return servo.getPortNumber();
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public Servo.Direction getDirection() {
        return servo.getDirection();
    }

    public void setPosition(double v) {
        servo.setPosition(v);
        if (limitsEnabled)
        {
            if (v > maximumSafetyPosition)
                v = maximumSafetyPosition;
            if (v < minimumSafetyPosition)
                v = minimumSafetyPosition;
        }
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void scaleRange(double v, double v1) {
        servo.scaleRange(v, v1);
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    public String getDeviceName() {
        return servo.getDeviceName();
    }

    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    public int getVersion() {
        return servo.getVersion();
    }

    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    public void close() {
        servo.close();
    }

    public void setMinimumSafetyPosition(double _minimumSafetyPosition) {
        minimumSafetyPosition = _minimumSafetyPosition;
        limitsEnabled = true;
    }

    public void setMaximumSafetyPosition(double _maximumSafetyPosition) {
        maximumSafetyPosition = _maximumSafetyPosition;
        limitsEnabled = true;
    }

    public void disableLimitChecks() {
        limitsEnabled = false;
    }

    public void enableLimitChecks() {
        limitsEnabled = true;
    }
}
