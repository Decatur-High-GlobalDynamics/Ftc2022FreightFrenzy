package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TeamOpMode extends LinearOpMode {
    public long targetLoopDuration_ms = 20;

    Robot robot;

    public long durationOfLastLoop_ms = -1;

    public TeamOpMode() {
        super();
        this.msStuckDetectInit=10000;
    }

    /**
     * This initializes and manages the overall OpMode. This is marked as final in order to
     * prevent casual interference.
     * There are the following methods subclasses can use to hook into the operation of the
     * OpMode:
     *    teamInit() : Called after the robot is initialized
     *    teamStart() : Called once after the Play button is pressed and the robot is notified that
     *                  the OpMode has started. teamStart() should return quickly; use teamRunOpMode()
     *                  if you want to take full control.
     *    teamRunOpMode() : Called after the Play button is pressed to pass control to the subclass
     *    teamLoop() : Called repeatedly if teamRunOpMode() is not overridden by a subclass.
     *
     * @throws InterruptedException
     */
    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        // Tell the subclass that the the OpMode is initialized.
        teamInit();

        robot.updateStatus("Waiting for start");
        while (!isStarted() && !isStopRequested()) {
            sleep(1);
            robot.grabbers_grab();
            robot.loop();
        }

        robot.updateStatus("Starting");
        robot.noteThatOpModeStarted();
        teamStart();

        robot.updateStatus("Running");
        teamRunOpMode();
    }


    public void teamSleep(long ms) throws InterruptedException {
        robot.sleep(ms);
    }

    /**
     * Subclasses can override this to be notified when the OpMode has been initialized,
     * after the robot object has been created. By default, this doesn't do anything.
     */
    protected void teamInit() { }

    /**
     * Subclasses can override this to be notified when the Play/Start button has be pressed.
     * By default, this doesn't do anything.
     */
    protected void teamStart() { }

    /**
     * Subclasses need to override this or teamLoop() to take over once the Play/Start button has been pressed.
     * By default, teamRunOpMode() runs teamLoop() repeatedly until the OpMode is supposed to stop. Therefore,
     * subclasses can be a looping OpMode or can take full control as a LinearOpMode.
     */
    protected void teamRunOpMode() throws InterruptedException
    {
        // Initialize this so the sleep() is 0
        long sleepBeforeStartingNextLoop_ms=0;
        while ( opModeIsActive() ) {
            robot.sleep(sleepBeforeStartingNextLoop_ms);

            long loopStartEpoch_ms = System.currentTimeMillis();
            teamLoop();
            long loopEndEpoch_ms = System.currentTimeMillis();

            // We want to run every targetLoopDuration_ms, so we see how long we took in the last loop
            // and sleep for the rest. Eg, if targetLoopDuration_ms is 20ms, and we took 15ms in the
            // last loop, we want to sleep 5ms before starting the next loop

            durationOfLastLoop_ms = loopEndEpoch_ms - loopStartEpoch_ms;
            if (durationOfLastLoop_ms > targetLoopDuration_ms)
                sleepBeforeStartingNextLoop_ms=0;
            else
                sleepBeforeStartingNextLoop_ms = targetLoopDuration_ms - durationOfLastLoop_ms;
        }
    }

    protected void teamLoop() throws InterruptedException
    {
    }
}
