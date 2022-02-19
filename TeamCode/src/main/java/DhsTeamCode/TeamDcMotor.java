package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Class to wrap a DcMotor, particularly with telemetry data,
 * including speed information that requires information to be saved
 * by the Motor between loops
 *
 */
public class TeamDcMotor {
    final Robot robot;
    final String name;
    final DcMotor motor;
    long previousReadingTime_ms;
    int previousPosition;
    int currentPosition;
    int speed_raw, speed_perSec;
    private Integer minimumSafetyPosition, maximumSafetyPosition;
    private Integer minPositionSeen, maxPositionSeen;
    public boolean limitsEnabled=false;

    private Long targetChangeTime_ms; // when the last order to go to a position was issued

    String alertString = null;
    Long alertStringTime_ms=null;

    public TeamDcMotor(Robot robot, OpMode opMode, String name) {
        this(robot, name, opMode.hardwareMap.dcMotor.get(name));
    }

    public TeamDcMotor(Robot robot, String name, DcMotor motor) {
        this.robot = robot;
        this.name = name;
        this.motor = motor;
        previousReadingTime_ms = System.currentTimeMillis();
        previousPosition = currentPosition = this.motor.getCurrentPosition();

        speed_raw = speed_perSec = 0;
    }

    public void loop() {
        long now_ms = System.currentTimeMillis();
        long elapsed_ms = now_ms - previousReadingTime_ms;
        previousReadingTime_ms = now_ms;

        previousPosition = currentPosition;
        currentPosition = motor.getCurrentPosition();
        speed_raw = currentPosition - previousPosition;

        // Make note of the min and max positions ever seen for this motor
        if ( minPositionSeen==null || minPositionSeen>currentPosition )
            minPositionSeen=currentPosition;
        if ( maxPositionSeen==null || maxPositionSeen<currentPosition )
            maxPositionSeen=currentPosition;

        if ( elapsed_ms == 0 )
            speed_perSec = 0;
        else
            speed_perSec = (int)(1000 * speed_raw/elapsed_ms);

        if ( limitsEnabled ) {
            if ( minimumSafetyPosition != null && motor.getPower()<0 && currentPosition <= minimumSafetyPosition) {
                motor.setPower(0);
                setAlert("Stopping at lower limit");
            }
            if ( maximumSafetyPosition != null && motor.getPower()>0 && currentPosition >= maximumSafetyPosition) {
                motor.setPower(0);
                setAlert("Stopping at upper limit");
            }
        }
    }

    public void setAlert(String messageFormat, Object... formatArgs) {
        alertString = String.format(messageFormat, formatArgs);
        alertStringTime_ms = System.currentTimeMillis();
    }

    public String getTelemetryString() {
        String alert="";
        if ( alertString!=null && (System.currentTimeMillis()-alertStringTime_ms)<5000)
            alert = "!!"+alertString+"!!";

        switch (motor.getMode()) {
            case RUN_TO_POSITION:
                if ( isBusy() )
                    return String.format("%sBusy %d-->%d (p%5.2f) (%d/s)",alert,
                            currentPosition, motor.getTargetPosition(), motor.getPower(), speed_perSec);
                else
                    return String.format("%sHold %5.2f hold@%d (%d/s)", alert,
                            motor.getPower(), currentPosition, speed_perSec);
            case RUN_USING_ENCODER:
                return String.format("%sspd %+5.2f @%d *%d/s*", alert, motor.getPower(), motor.getCurrentPosition(), speed_perSec);
            case RUN_WITHOUT_ENCODER:
                return String.format("%spow %+5.2f @%d (%d/s)", alert, motor.getPower(), motor.getCurrentPosition(), speed_perSec);
            case STOP_AND_RESET_ENCODER:
                return String.format("%sreset %+5.2f @%d (%d/s)", alert, motor.getPower(), motor.getCurrentPosition(), speed_perSec);
            default:
                return String.format("%sunk %+5.2f @%d (%d/s)", alert, motor.getPower(), motor.getCurrentPosition(), speed_perSec);

        }
    }

    public int getMotorPositionAboveMinimumSeen() {
        return currentPosition-minPositionSeen;
    }

    public int getMotorPositionBelowMaximumSeen() {
        return maxPositionSeen-currentPosition;
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see #setPower(double)
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link DcMotor.RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    public void setTargetPosition(int position) {
        targetChangeTime_ms = System.currentTimeMillis();
        motor.setTargetPosition(position);
    }

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    public boolean isBusy() {
        if ( getMode() != DcMotor.RunMode.RUN_TO_POSITION )
            return false;

        if ( targetChangeTime_ms == null )
            return motor.isBusy();

        long timeSinceTargetChange_ms = System.currentTimeMillis() - targetChangeTime_ms;

        // Consider the motor unbusy if it isn't moving
        if ( timeSinceTargetChange_ms>1000 && Math.abs(speed_perSec) < 500 )
            return false;
        else
            return motor.isBusy();
    }

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     * @return the current reading of the encoder for this motor
     * @see #getTargetPosition()
     * @see DcMotor.RunMode#STOP_AND_RESET_ENCODER
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Sets the current run mode for this motor
     * @param mode the new current run mode for this motor
     * @see DcMotor.RunMode
     * @see #getMode()
     */
    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);

        if ( mode != DcMotor.RunMode.RUN_TO_POSITION )
            targetChangeTime_ms = null;

        // If the encoder is being reset, then note that the min/max positions have changed
        if ( mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER ) {
            minPositionSeen = maxPositionSeen = 0;
        }
    }

    /**
     * Returns the current run mode for this motor
     * @return the current run mode for this motor
     * @see DcMotor.RunMode
     */
    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     *
     * @see #getDirection()
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     * @return the current logical direction in which this motor is set as operating.
     */
    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum
     * possible power / speed supported according to the run mode in which the
     * motor is operating.
     *
     * <p>Setting a power level of zero will brake the motor</p>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @see #getPower()
     * @see DcMotor#setMode(DcMotor.RunMode)
     * @see DcMotor#setPowerFloat()
     */
    public void setPower(double power) {
        if (limitsEnabled)
        {
            if (minimumSafetyPosition != null && power < 0 && currentPosition <= minimumSafetyPosition) {
                power = 0;
                setAlert("Can't go below lower limit: %d <= %d", currentPosition, minimumSafetyPosition);
            }
            if (minimumSafetyPosition != null && power > 0 && currentPosition >= maximumSafetyPosition) {
                power = 0;
                setAlert("Can't go beyond upper limit: %d >= %d", currentPosition, maximumSafetyPosition);
            }
        }
        motor.setPower(power);
    }

    /**
     * Returns the current configured power level of the motor.
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    public double getPower() {
        return motor.getPower();
    }

    public void setMinimumSafetyPosition(int _minimumSafetyPosition) {
        minimumSafetyPosition = _minimumSafetyPosition;
        limitsEnabled = true;
    }

    public void setMaximumSafetyPosition(int _maximumSafetyPosition) {
        maximumSafetyPosition = _maximumSafetyPosition;
        limitsEnabled = true;
    }

    public void disableLimitChecks() {
        limitsEnabled=false;
    }

    public void enableLimitChecks() {
        limitsEnabled=true;
    }

    public void waitForMotorToReachTarget() throws InterruptedException {
        while ( isBusy() )
            robot.sleep(5);
    }
}
