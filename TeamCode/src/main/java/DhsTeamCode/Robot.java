package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import androidx.annotation.NonNull;

public class Robot {

    // Safety values to keep robot from falling over
    public final double FORWARD_TILT_WARNING_ANGLE=5, FORWARD_TILT_PANIC = 13.0; // From experimenting with robot
    public final double MAX_BACKWARD_POWER_WHEN_TILTED_SLIGHTLY = -0.2; // From rough experiments
    public static final double TILT_PANIC_RECOVERY_POWER = 0;
    private final TouchSensor frontTouch;
    public ColorSensor sensorColor;
    private final TouchSensor armTouch;

    // Sensor on tips of grabbers that indicate if grabbers are touching with opposite logic:
    //    TRUE: NOT Touching
    //    FALSE: Touching
    private final DigitalChannel grabberTouchSensor;

    public boolean tiltprotectionenabled=true;

    public final double TICKS_PER_INCH = 3750/42.5;
    private Long grabTime_ms = null;

    public enum ARM_PRESET {
        TOP(200),
        HIGH(-1035),
        MID(-1760),
        GRAB(-2450),
        LOWER_LIMIT(GRAB.motorPosition-200),
        DOWN(-2962);

        final int motorPosition;

        ARM_PRESET(int motorPosition) {
            this.motorPosition = motorPosition;
        }


        ARM_PRESET getNextPreset_below() {
            // Are we at the bottom already?
            if ( this == DOWN )
                return DOWN;

            return ARM_PRESET.values()[ordinal() +1];
        }
        ARM_PRESET getNextPreset_above() {
            // Are we at the top already?... If so, we'll move the arm straight up
            if ( this == TOP )
                return TOP;
            return ARM_PRESET.values()[ordinal() -1];
        }

        @NonNull
        @Override
        public String toString() {
            return String.format("%s#%d", name().toLowerCase(), ordinal());
        }
    }

    private ARM_PRESET currentArmPreset =null;

    public final double ARM_UP_SPEED_WHEN_RESETTING=0.20;
    public final double ARM_SPEED_SET_POSITION=0.25, ARM_SPEED_UP=0.20, ARM_SPEED_DOWN=0.10;
    public final double ARM_HOLD_POWER=0.5;
    private boolean currentlyHoldingArmPosition=false;

    public final Servo rightGrabber;
    public final Servo leftGrabber;

    // This is a ColorSensor that we just use a bright LED to indicate if a block has been grabbed
    ColorSensor grabIndicator;

    // Start the grabber folded back
    public final double LEFTGRABBER_POSITION_PARKED =1.0;
    public final double RIGHTGRABBER_POSITION_PARKED =0.0;

    // Grabbers are straight sideways
    public final double LEFTGRABBER_POSITION_SIDEWAYS  =0.5;
    public final double RIGHTGRABBER_POSITION_SIDEWAYS =0.5;

    // Grabbers are holding a block
    public final double LEFTGRABBER_POSITION_GRAB =0.05;
    public final double RIGHTGRABBER_POSITION_GRAB=0.95;

    TeamOpMode opMode;
    TeamDcMotor leftDrive, rightDrive;
    TeamDcMotor armLiftMotor;
    TeamDcMotor turn_table;

    // Is Arm in front?
    //   The left/right motors are defined by arm being in front
    //   but this boolean switches this.
    boolean armIsInFrontOfRobot=true;

    long lastLoopStartTime_ms = System.currentTimeMillis();

    // Gyro variables
    float heading_totalDegreesTurned, desiredHeading_totalDegreesTurned;
    float lastImuReading1, lastImuReading2, lastImuReading3;

    BNO055IMU imu;
    long initializedTime_ms = System.currentTimeMillis();
    long opModeStarted_ms=-1;
    String status="";
    long statusChangedTime_ms = System.currentTimeMillis();

    public Robot(TeamOpMode _opMode) throws InterruptedException {
        opMode = _opMode;
        updateStatus("initializing");

        rightDrive= new TeamDcMotor(this, opMode, "right_drive");
        rightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive=new TeamDcMotor(this, opMode, "left_drive");

        armLiftMotor =new TeamDcMotor(this, opMode, "cage_lift");
        armLiftMotor.setMaximumSafetyPosition(ARM_PRESET.TOP.motorPosition);
        armLiftMotor.setMinimumSafetyPosition(ARM_PRESET.DOWN.motorPosition);

        turn_table=new TeamDcMotor(this, opMode, "turn_table");
        turn_table.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightGrabber = opMode.hardwareMap.servo.get("left_grabber");
        leftGrabber = opMode.hardwareMap.servo.get("right_grabber");

        frontTouch = opMode.hardwareMap.touchSensor.get("front_touch");
        armTouch = opMode.hardwareMap.touchSensor.get("arm touch");

        grabberTouchSensor = opMode.hardwareMap.digitalChannel.get("digital2");
        grabberTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        lastImuReading1 = heading_totalDegreesTurned = _getHeadingFromImu();
        sensorColor = opMode.hardwareMap.get(ColorSensor.class, "revcolor");


        // Color sensors have really bright LEDs, so we're using one to indicate if a block is being held
        grabIndicator = opMode.hardwareMap.get(ColorSensor.class, "grab indicator");
        grabIndicator.enableLed(false);

        opMode.telemetry.addData("R", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        long now_ms = System.currentTimeMillis();
                        return String.format("t_init=%3ds t_run=%3ds %4d loops/s|%s (%3ds)",
                                (now_ms - initializedTime_ms)/1000,
                                opModeStarted_ms==-1 ? -1 : (now_ms-opModeStarted_ms)/1000,
                                opMode.durationOfLastLoop_ms == 0 ? 1000 : 1000/opMode.durationOfLastLoop_ms,
                                status, (now_ms - statusChangedTime_ms)/1000);
                    }
                });

        opMode.telemetry.addData("G1", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        Gamepad gp = opMode.gamepad1;
                        return String.format("%s:%+3.1f,%+3.1f|%s:%+3.1f,%+3.1f|%s%s%s%s|%s%s%s%s|%s %s|t:%3.1f/%3.1f",
                                gp.left_stick_button ? "LJ" : "lj",gp.left_stick_x, gp.left_stick_y,
                                gp.right_stick_button ? "RJ" : "rj", gp.right_stick_x, gp.right_stick_y,
                                gp.a ? "A" : "a", gp.b ? "B" : "b", gp.x ? "X" : "x", gp.y ? "Y" : "y",
                                gp.dpad_up ? "U" : "u", gp.dpad_down ? "D" : "d", gp.dpad_left ? "L" : "l", gp.dpad_right ? "R" : "r",
                                gp.left_bumper ? "LB" : "lb", gp.right_bumper ? "RB" : "rb",
                                gp.left_trigger, gp.right_trigger);
                    }
                });

        opMode.telemetry.addData("G2", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        Gamepad gp = opMode.gamepad2;
                        return String.format("%s:%+3.1f,%+3.1f|%s:%+3.1f,%+3.1f|%s%s%s%s|%s%s%s%s|%s %s|t:%3.1f/%3.1f",
                                gp.left_stick_button ? "LJ" : "lj",gp.left_stick_x, gp.left_stick_y,
                                gp.right_stick_button ? "RJ" : "rj", gp.right_stick_x, gp.right_stick_y,
                                gp.a ? "A" : "a", gp.b ? "B" : "b", gp.x ? "X" : "x", gp.y ? "Y" : "y",
                                gp.dpad_up ? "U" : "u", gp.dpad_down ? "D" : "d", gp.dpad_left ? "L" : "l", gp.dpad_right ? "R" : "r",
                                gp.left_bumper ? "LB" : "lb", gp.right_bumper ? "RB" : "rb",
                                gp.left_trigger, gp.right_trigger);
                    }
                });

        opMode.telemetry.addData("Drive", "%s",
                        new Func<String>() {
                            @Override
                            public String value() {
                                return String.format("Left=%s|Right=%s",
                                        armIsInFrontOfRobot ? leftDrive.getTelemetryString() : rightDrive.getTelemetryString(),
                                        armIsInFrontOfRobot ? rightDrive.getTelemetryString() : leftDrive.getTelemetryString());
                            }
                        });

        opMode.telemetry.addData("IMU", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("hdg=%+.1f | goal=%+.1f | err=%+.1f | raw=%+.1f/%+.1f/%+.1f",
                                heading_totalDegreesTurned, desiredHeading_totalDegreesTurned,
                                desiredHeading_totalDegreesTurned - heading_totalDegreesTurned,
                                lastImuReading1, lastImuReading2, lastImuReading3);
                    }
                });
        opMode.telemetry.addData("Arm", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("LiftMotor: %s [%s]%s",
                                armLiftMotor.getTelemetryString(), currentArmPreset,
                                armTouch.isPressed() ? " LIMIT" : "");
                    }
                });

        opMode.telemetry.addData("Grab", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("L/R: %.2f / %.2f | dur_s: %.1f | Blk: %s | Touch: %s",
                                leftGrabber.getPosition(),
                                rightGrabber.getPosition(),
                                grabTime_ms==null ? -1 : 1.0*(System.currentTimeMillis()-grabTime_ms)/1000,
                                blockIsProbablyGrabbed() ? "YES" : "no",
                                grabbersAreTouchingEachOther() ? "yes" : "no");
                    }
                });
        opMode.telemetry.addData("Turntable", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return turn_table.getTelemetryString();
                    }
                });
        opMode.telemetry.addData("Sensors", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("Front: %s | Color: %3d/%3d/%3d" ,
                                frontTouch.isPressed() ? "PRESSED" : "unpressed",
                                grabIndicator.red(), grabIndicator.green(), grabIndicator.blue());
                    }
                });
        resetArm();
        sleep(100);
        updateStatus("initialized");

    }

    private float _getHeadingFromImu() {
        return imu.getAngularOrientation().firstAngle;
    }

    private float _getAngle2FromImu() {
        return imu.getAngularOrientation().secondAngle;
    }
    private float _getAngle3FromImu() {
        return imu.getAngularOrientation().thirdAngle;
    }
    /**
     * Set the left power, based on whether robot is reversed
     * @param power
     */
    public void setLeftPower(double power) {
        if (tiltprotectionenabled) {
            // the robot falls when it's going backwards too fast (the power is negative)
            if (lastImuReading2 > FORWARD_TILT_PANIC && power <= TILT_PANIC_RECOVERY_POWER)
                power = TILT_PANIC_RECOVERY_POWER;
            else if (lastImuReading2 > FORWARD_TILT_WARNING_ANGLE && power < MAX_BACKWARD_POWER_WHEN_TILTED_SLIGHTLY)
                power = MAX_BACKWARD_POWER_WHEN_TILTED_SLIGHTLY;
        }
        if ( armIsInFrontOfRobot )
            leftDrive.setPower(power);
        else
            rightDrive.setPower(-power);
    }

    /**
     * set the right power, based on whether robot is reversed
     * @param power
     */
    public void setRightPower(double power) {
        // the robot falls when it's going backwards too fast (the power is negative)
        if (tiltprotectionenabled) {
            if (lastImuReading2 > FORWARD_TILT_PANIC && power <= TILT_PANIC_RECOVERY_POWER)
                power = TILT_PANIC_RECOVERY_POWER;
            else if (lastImuReading2 > FORWARD_TILT_WARNING_ANGLE && power < MAX_BACKWARD_POWER_WHEN_TILTED_SLIGHTLY)
                power = MAX_BACKWARD_POWER_WHEN_TILTED_SLIGHTLY;
        }
        if ( armIsInFrontOfRobot )
            rightDrive.setPower(power);
        else
            leftDrive.setPower(-power);
    }

    public void setDrivePower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    public void noteThatOpModeStarted() throws InterruptedException {
        opModeStarted_ms = System.currentTimeMillis();
    }

    // Describe what the robot is doing
    public void updateStatus(String statusFormat, Object... formatArgs) {
        String newStatus = String.format(statusFormat, formatArgs);
        if ( !status.equals(newStatus) ) {
            status = newStatus;
            statusChangedTime_ms = System.currentTimeMillis();
        }
    }

    public void sleep(long mSec) throws InterruptedException {
        long start = System.currentTimeMillis();
        long end   = start + mSec;

        while ( opMode.opModeIsActive() && System.currentTimeMillis() <= end ) {
            loop();
            Thread.sleep(5);
        }
        // Make sure we do at least one loop
        loop();
    }

    public void goForward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition+inches*TICKS_PER_INCH);
        setDrivePower(0.70);

        while(leftDrive.getCurrentPosition()<endPosition){
            sleep(10);
        }
        setDrivePower(0);
    }

    public void goBackward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition-inches*TICKS_PER_INCH);
        setDrivePower(-0.70);

        while(leftDrive.getCurrentPosition()>endPosition){
            sleep(10);
        }
        setDrivePower(0);
    }
    public void turnLeft(float degrees) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        double startHeading= heading_totalDegreesTurned;
        double endHeading= startHeading-degrees;
        setLeftPower(-0.30);setRightPower(0.30);

        while(heading_totalDegreesTurned>endHeading){
            sleep(10);
        }
        setDrivePower(0);
    }
    public void turnRight(float degrees) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        double startHeading= heading_totalDegreesTurned;
        double endHeading= startHeading+degrees;
        setLeftPower(0.30);setRightPower(-0.30);

        while(heading_totalDegreesTurned<endHeading){
            sleep(10);
        }
        setDrivePower(0);
    }




    private void resetArm() throws InterruptedException {
        armLiftMotor.disableLimitChecks();
        setArmPower(ARM_UP_SPEED_WHEN_RESETTING);
        // Wait for arm to reach upper limit sensor
        while ( !armTouch.isPressed() ) {
            sleep(10);
        }

        // Stop and let arm stabilize
        holdArmAtPosition();
        sleep(500);
        currentArmPreset = ARM_PRESET.TOP;

        // Reset encoder so it's 0 at rest on the limit switch
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.enableLimitChecks();

        // Give team a moment to get block into jaws
        sleep(2000);

        // Set the grabbers to hold block
        grabbers_grab();
        // Trying a second grabbers_grab because the one of the grabbers sometimes doesn't engage!
        sleep(1000);
        grabbers_grab();
    }


    public void setArmPosition(ARM_PRESET armPosition) {
        currentArmPreset = armPosition;
        setArmPosition(true, armPosition.motorPosition);
    }

    private void setArmPosition(boolean basedOnPresetPosition, int position) {
        // If this isn't based on an ARM_POSITION, then we're no longer at a position
        if ( !basedOnPresetPosition )
            currentArmPreset = null;

        currentlyHoldingArmPosition=false;
        armLiftMotor.setTargetPosition(position);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftMotor.setPower(ARM_SPEED_SET_POSITION);
    }

    public void holdArmAtPosition() {
        // Relax motor if armTouch is pressed, otherwise hold at current position
        if ( armTouch.isPressed() ) {
            currentlyHoldingArmPosition = true;
            armLiftMotor.setPower(0);
            return;
        }

        // only set the target position once, otherwise wiggles become the new set points
        if ( !currentlyHoldingArmPosition  ) {
            currentlyHoldingArmPosition = true;

            // We're going to wait to reach our target position if we're on our way there
            if ( !armLiftMotor.isBusy() ) {
                armLiftMotor.setTargetPosition(armLiftMotor.getCurrentPosition());
                armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLiftMotor.setPower(ARM_HOLD_POWER);
            }
        }
    }


    private void setArmPower(double power) {
        if ( power==0 ) {
            holdArmAtPosition();
        } else {
            currentArmPreset =null;
            currentlyHoldingArmPosition=false;
            armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armLiftMotor.setPower(power);
        }
    }

    public void moveArm_presetUp() {
        if ( currentArmPreset == null )
            setArmPosition(ARM_PRESET.HIGH);
        else if ( !armLiftMotor.isBusy() )
            // move to the next position only if the motor has reached its preset goal
            setArmPosition(currentArmPreset.getNextPreset_above());
    }

    public void moveArm_presetDown() {
        if ( currentArmPreset == null )
            setArmPosition(ARM_PRESET.GRAB);
        else if ( !armLiftMotor.isBusy() )
            // move to the next position only if the motor has reached its preset goal
            setArmPosition(currentArmPreset.getNextPreset_below());
    }

    public void moveArm_up() {
        // Ignore requests to move arm passed limit switch
        if ( armTouch.isPressed() ) {
            return;
        }

        setArmPower(+ARM_SPEED_UP);
    }

    public void moveArm_down() {
        // Turn off motor below the lower limit
        if ( armLiftMotor.getCurrentPosition() < ARM_PRESET.LOWER_LIMIT.motorPosition )
            setArmPower(0);
        else
            setArmPower(-ARM_SPEED_DOWN);
    }

    public void loop() {
        long now_ms = System.currentTimeMillis();
        lastLoopStartTime_ms = now_ms;

        // Update heading
        float imuReading = _getHeadingFromImu();
        float degreesTurned = imuReading - lastImuReading1;
        lastImuReading1 = imuReading;
        lastImuReading2 = _getAngle2FromImu();
        lastImuReading3 = _getAngle3FromImu();

        // Check for wrapping around (suddenly we will have turned a lot)
        if ( degreesTurned > 180 )
            degreesTurned -= 360;
        else if ( degreesTurned < -180 )
            degreesTurned += 360;

        heading_totalDegreesTurned += degreesTurned;

        // Protect against arm moving past limit switch
        if ( armTouch.isPressed() && armLiftMotor.speed_perSec>0 )
            setArmPower(0);

        if (  armNeedsGrabbersRetracted() )
            grabbers_park();

        leftDrive.loop();
        rightDrive.loop();
        armLiftMotor.loop();
        turn_table.loop();

        grabIndicator.enableLed(blockIsProbablyGrabbed());

        opMode.telemetry.update();
    }

    public void setTurntablePower(double percent) {
        final double MAX_POWER = 1.0;

        turn_table.setPower(MAX_POWER * percent);
        turn_table.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the position of the grabber. 1 is fully out, 0 is fully in/grabbing.
     * @param position
     */
    public void grabber_setPosition(double position) {
        // Make sure position is positive
        position=Math.abs(position);
        rightGrabber.setPosition(Range.clip(1-position, 0,1));
        leftGrabber.setPosition(Range.clip(position, 0, 1));

        if ( position<0.1 )
            grabTime_ms = System.currentTimeMillis();
        else
            grabTime_ms = null;
    }

    public void grabbers_park() {
        grabTime_ms = null;
        leftGrabber.setPosition(LEFTGRABBER_POSITION_PARKED);
        rightGrabber.setPosition(RIGHTGRABBER_POSITION_PARKED);
    }

    public void grabbers_release() {
        if ( armNeedsGrabbersRetracted() )
            return;

        grabTime_ms = null;
        leftGrabber.setPosition(RIGHTGRABBER_POSITION_SIDEWAYS);
        rightGrabber.setPosition(LEFTGRABBER_POSITION_SIDEWAYS);
    }

    public void grabbers_grab() {
        if ( armNeedsGrabbersRetracted() )
            return;

        leftGrabber.setPosition(LEFTGRABBER_POSITION_GRAB);
        rightGrabber.setPosition(RIGHTGRABBER_POSITION_GRAB);
        if ( grabTime_ms == null )
            grabTime_ms = System.currentTimeMillis();
    }

    private boolean armNeedsGrabbersRetracted() {
        return armLiftMotor.getCurrentPosition() < ARM_PRESET.LOWER_LIMIT.motorPosition;
    }

    public boolean blockIsProbablyGrabbed() {
        // This will be null when we're not trying to grab
        if ( grabTime_ms==null )
            return false;

        // See how long since we grabbed... this way we know if the grabbers should have closed
        // If they're still open after this close duration, then there is probably a block there
        long grabDurationTime_ms = System.currentTimeMillis()-grabTime_ms;

        // Estimating that it takes 1/2 second to fully close
        if ( grabDurationTime_ms < 500 )
            return false;

        return !grabbersAreTouchingEachOther();
    }

    public boolean grabbersAreTouchingEachOther() {
        // The grabber touch sensor returns opposite logic
        return grabberTouchSensor.getState() == false;
    }
}

