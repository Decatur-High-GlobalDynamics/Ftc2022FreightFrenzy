package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

public class Bert_Robot {
    // Safety values to keep robot from falling over
    public final double MAX_FORWARD_TILT = 13.0; // From experimenting with robot
    public final double MAX_BACKWARD_POWER_WHEN_TILTED = 0.2; // From rough experiments

    public final double TICKS_PER_INCH = 10000/84.75;

    public final int ARM_BOTTOM_POSITION=0,
                     ARM_LOW_POSITION=1021,
                     ARM_MID_POSITION=1321,
                     ARM_HIGH_POSITION=1621,
                     ARM_LIMIT_POSITION = 1800;

    public final double ARM_DOWN_SPEED_WHEN_RESETTING=-0.10;
    public final double ARM_LIFT_SPEED = 0.30;
    public final double ARM_DOWN_SPEED = -0.20;
    public final Servo leftWhisker;
    public final Servo rightWhisker;
    public final double LEFTWHISKER_STARTPOSITION=0;
    public final double RIGHTWHISKER_STARTPOSITION=1.0;
    public final double RIGHTWHISKER_MAXPOSITION=.2;
    public final double LEFTWHISKER_MAXPOSITION=.6;
    public final double LEFTWHISKER_MIDPOSITION=.3;
    public final double RIGHTWHISKER_MIDPOSITION=.5;

    public final double WHISKER_SPEED =0.05;

    OpMode opMode;
    BertDcMotor leftDrive, rightDrive;
    BertDcMotor liftMotor, frontDoor;
    Servo backDoor;
    BertDcMotor turn_table;

    // Is Arm in front?
    //   The left/right motors are defined by arm being in front
    //   but this boolean switches this.
    boolean armIsInFrontOfRobot=true;

    long lastLoopTime_ms = System.currentTimeMillis();

    // Gyro variables
    float heading_totalDegreesTurned, desiredHeading_totalDegreesTurned;
    float lastImuReading1, lastImuReading2, lastImuReading3;

    BNO055IMU imu;
    long initializedTime_ms = System.currentTimeMillis();
    long opModeStarted_ms=-1;
    String status="";
    long statusChangedTime_ms = System.currentTimeMillis();

    public Bert_Robot(OpMode _opMode) {
        opMode = _opMode;
        rightDrive= new BertDcMotor(opMode, "right_drive");
        rightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive=new BertDcMotor(opMode, "left_drive");

        liftMotor=new BertDcMotor(opMode, "cage_lift");
        liftMotor.setMinimumPosition(0);
        liftMotor.setMaximumPosition(ARM_LIMIT_POSITION);

        turn_table=new BertDcMotor(opMode, "turn_table");
        turn_table.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontDoor=new BertDcMotor(opMode, "front_door");
        backDoor  = opMode.hardwareMap.servo.get("back_door");

        leftWhisker  = opMode.hardwareMap.servo.get("left_whisker");
        rightWhisker  = opMode.hardwareMap.servo.get("right_whisker");

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        lastImuReading1 = heading_totalDegreesTurned = _getHeadingFromImu();

        opMode.telemetry.addData("Robot", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        long now_ms = System.currentTimeMillis();
                        return String.format("t=%3ds/%3ds|%s (%3ds)",
                                (now_ms - initializedTime_ms)/1000,
                                opModeStarted_ms==-1 ? -1 : (now_ms-opModeStarted_ms)/1000,
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
                        return String.format("hdg=%+6.1f | goal=%+6.1f | err=%+3.0f | raw=%+6.1f/%+6.1f/%+6.1f",
                                heading_totalDegreesTurned, desiredHeading_totalDegreesTurned,
                                desiredHeading_totalDegreesTurned - heading_totalDegreesTurned,
                                lastImuReading1, lastImuReading2, lastImuReading3);
                    }
                });
        opMode.telemetry.addData("Arm", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("Lift: %s | F: %s |  B: %.2f",
                                liftMotor.getTelemetryString(),
                                frontDoor.getTelemetryString(),
                                backDoor.getPosition());
                    }
                });
        opMode.telemetry.addData("Whiskers", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("Left: %.2f | Right: %.2f" ,
                                leftWhisker.getPosition(),
                                rightWhisker.getPosition());


                    }
                });

        resetArm();
        sleep(100);
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
        // the robot falls when it's going backwards too fast (the power is negative)
        if (  lastImuReading2>MAX_FORWARD_TILT && power < -MAX_BACKWARD_POWER_WHEN_TILTED)
            power = -MAX_BACKWARD_POWER_WHEN_TILTED;

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
        if (  lastImuReading2>MAX_FORWARD_TILT && power < -MAX_BACKWARD_POWER_WHEN_TILTED)
            power = -MAX_BACKWARD_POWER_WHEN_TILTED;

        if ( armIsInFrontOfRobot )
            rightDrive.setPower(power);
        else
            leftDrive.setPower(-power);
    }

    public void setDrivePower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    public void noteThatOpModeStarted() {
        if ( opModeStarted_ms == -1 )
            opModeStarted_ms = System.currentTimeMillis();
    }

    // Describe what the robot is doing
    public void updateStatus(String statusFormat, Object... formatArgs) {
        String newStatus = String.format(statusFormat, formatArgs);
        if ( !status.equals(newStatus) ) {
            status = newStatus;
            statusChangedTime_ms = System.currentTimeMillis();
        }
        loop();
    }

    public void sleep(long mSec) {
        long start = System.currentTimeMillis();
        long end   = start + mSec;

        while ( System.currentTimeMillis() <= end ) {
            loop();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException("Interrupted");
            }
        }
        // Make sure we do at least one loop
        loop();
    }

    public void goForward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition+inches*TICKS_PER_INCH);
        setDrivePower(.40);

        while(leftDrive.getCurrentPosition()<endPosition){
            sleep(10);
        }
        setDrivePower(0);
    }

    public void goBackward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition-inches*TICKS_PER_INCH);
        setDrivePower(-.40);

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




    private void resetArm() {
        liftMotor.disableLimitChecks();
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(ARM_DOWN_SPEED_WHEN_RESETTING);
        // let the arm get moving
        sleep(100);

        // When to consider the arm-lift motor to have stopped
        final int stoppedThreshold=25;
        while ( liftMotor.speed_perSec > stoppedThreshold ) {
            sleep(100);
        }

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        holdArmPosition();
        liftMotor.enableLimitChecks();

        // Reset the arm whiskers, but first move the front dorr out of the way
        frontDoor.setPower(0.1);
        sleep(1500);
        moveBothWhiskersFullyIn();
        frontDoor.setPower(-0.1);
        sleep(1000);
    }


    public void setArmPosition(int position) {
        if ( position < liftMotor.getCurrentPosition() )
            liftMotor.setPower(0.3);
        else
            liftMotor.setPower(0.5);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void holdArmPosition() {
        if (liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            setArmPosition(liftMotor.getCurrentPosition());
    }


    private void setArmPower(double power) {
        if ( power==0 ) {
            holdArmPosition();
            return;
        }

        if (liftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setPower(power);
    }

    public void moveArmUp() {
        setArmPower(ARM_LIFT_SPEED);
    }

    public void moveArmDown() {
        if ( liftMotor.getCurrentPosition()<500 )
            setArmPower(ARM_DOWN_SPEED_WHEN_RESETTING);
        else if (liftMotor.getCurrentPosition() < 1000 )
            setArmPower( (ARM_DOWN_SPEED_WHEN_RESETTING+ARM_DOWN_SPEED)/2);
        else
            setArmPower(ARM_DOWN_SPEED);
    }

    public void backDoorClose() {
        backDoor.setPosition(0.3);
    }

    public void backDoorOpen() {
        backDoor.setPosition(0);
    }

    public void loop() {
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

        leftDrive.loop();
        rightDrive.loop();
        liftMotor.loop();
        frontDoor.loop();
        turn_table.loop();
    }

    public void setTurntablePower(double percent) {
        final double MAX_POWER=1.0;

        turn_table.setPower(MAX_POWER*percent);
        turn_table.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveBothWhiskersIn() {
        if ( leftWhisker.getPosition() < LEFTWHISKER_MIDPOSITION )
            leftWhisker.setPosition(LEFTWHISKER_MAXPOSITION);
        if ( rightWhisker.getPosition() > RIGHTWHISKER_MIDPOSITION )
            rightWhisker.setPosition(RIGHTWHISKER_MAXPOSITION);

        leftWhisker.setPosition(Range.clip(leftWhisker.getPosition() - WHISKER_SPEED, LEFTWHISKER_MIDPOSITION, LEFTWHISKER_MAXPOSITION));
        rightWhisker.setPosition(Range.clip(rightWhisker.getPosition()+ WHISKER_SPEED, RIGHTWHISKER_MAXPOSITION, RIGHTWHISKER_MIDPOSITION));
    }

    public void moveBothWhiskersOut() {
        if ( leftWhisker.getPosition() < LEFTWHISKER_MIDPOSITION )
            leftWhisker.setPosition(LEFTWHISKER_MAXPOSITION);
        if ( rightWhisker.getPosition() > RIGHTWHISKER_MIDPOSITION )
            rightWhisker.setPosition(RIGHTWHISKER_MAXPOSITION);

        leftWhisker.setPosition(Range.clip(leftWhisker.getPosition() + WHISKER_SPEED, LEFTWHISKER_MIDPOSITION, LEFTWHISKER_MAXPOSITION));
        rightWhisker.setPosition(Range.clip(rightWhisker.getPosition()- WHISKER_SPEED, RIGHTWHISKER_MAXPOSITION, RIGHTWHISKER_MIDPOSITION));
    }

    public void parkWhiskers() {
        leftWhisker.setPosition(LEFTWHISKER_STARTPOSITION);
        rightWhisker.setPosition(RIGHTWHISKER_STARTPOSITION);
    }

    public void moveBothWhiskersFullyOut() {
        leftWhisker.setPosition(LEFTWHISKER_MAXPOSITION);
        rightWhisker.setPosition(RIGHTWHISKER_MAXPOSITION);
    }
    public void moveBothWhiskersFullyIn() {
        leftWhisker.setPosition(LEFTWHISKER_STARTPOSITION);
        rightWhisker.setPosition(RIGHTWHISKER_STARTPOSITION);
    }
}

