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
public class BertDcMotor  {
    String name;
    DcMotor motor;
    long previousReadingTime_ms;
    int previousPosition;
    int currentPosition;
    int speed_raw, speed_perSec;
    private Integer minimumPosition, maximumPosition;
    private boolean limitsEnabled=false;

    public BertDcMotor(OpMode opMode, String name) {
        this(name, opMode.hardwareMap.dcMotor.get(name));
    }

    public BertDcMotor(String name, DcMotor motor) {
        this.name = name;
        this.motor =motor;
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

        if ( elapsed_ms == 0 )
            speed_perSec = 0;
        else
            speed_perSec = (int)(1000 * speed_raw/elapsed_ms);

        if ( limitsEnabled ) {
            if ( minimumPosition != null && motor.getPower()<0 && currentPosition<=minimumPosition )
                motor.setPower(0);
            if ( maximumPosition != null && motor.getPower()>0 && currentPosition>=maximumPosition)
                motor.setPower(0);
        }
    }

    public String getTelemetryString() {
        switch (motor.getMode()) {
            case RUN_TO_POSITION:
                if ( isBusy() )
                    return String.format("Busy %d-->%d (%5.2f) (%d/s)",
                            currentPosition, motor.getTargetPosition(), motor.getPower(), speed_perSec);
                else
                    return String.format("%5.2f hold@%d (%d/s)",
                            motor.getPower(), currentPosition, speed_perSec);

            case RUN_USING_ENCODER:
                return String.format("s=%+5.2f (%d/s)", motor.getPower(), speed_perSec);
            default:
                return String.format("%+5.2f (%d/s)", motor.getPower(), speed_perSec);
        }
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

    public void setMinimumPosition(int _minimumPosition) {
        minimumPosition = _minimumPosition;
        limitsEnabled = true;
    }

    public void setMaximumPosition(int _maximumPosition) {
        maximumPosition = _maximumPosition;
        limitsEnabled = true;
    }

    public void disableLimitChecks() {
        limitsEnabled=false;
    }

    public void enableLimitChecks() {
        limitsEnabled=true;
    }
}
