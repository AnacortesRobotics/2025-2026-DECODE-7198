package org.firstinspires.ftc.teamcode.Commands;

/**
 * <code>WaitCommand</code> is a non-blocking command that allows a specified amount
 * of time to pass before finishing.
 * <p>
 * Use this in a CommandGroup to create delays between robot actions (e.g., waiting
 * for an intake to spin up).
 */
public class WaitCommand extends Command {

    /**
     * The total amount of time, in milliseconds, that the command should wait.
     */
    private long msDuration = 0;

    /**
     * The system timestamp (in milliseconds) recorded when the command started.
     * This is used as the reference point to calculate elapsed time.
     */
    private long startTime = 0;

    /**
     * Creates a new WaitCommand.
     * This command will do nothing, and end after the specified duration.
     *
     * @param duration The time to wait in milliseconds (e.g., 1000 for 1 second).
     */
    public WaitCommand(long duration) {
        this.msDuration = duration;
    }

    /**
     * Initializes the command by capturing the current system time.
     */
    @Override
    public void init() {
        startTime = System.currentTimeMillis();
    }

    /**
     * Evaluates whether the duration has expired.
     *
     * @return <b>true</b> If elapsed time exceeds duration,
     * <b>false</b> otherwise.
     */
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= startTime + msDuration;
    }

}

