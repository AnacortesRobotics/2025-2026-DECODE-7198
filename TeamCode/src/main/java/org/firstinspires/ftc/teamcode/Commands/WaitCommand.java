package org.firstinspires.ftc.teamcode.Commands;

public class WaitCommand extends Command {

    private long msDuration = 0;
    private long startTime = 0;

    public WaitCommand(long duration) {
        this.msDuration = duration;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= startTime + msDuration;
    }

}
