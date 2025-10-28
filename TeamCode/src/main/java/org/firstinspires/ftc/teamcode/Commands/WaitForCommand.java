package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.List;

public class WaitForCommand extends Command {

    private Command commandToRun;
    private Command commandToWait;

    public WaitForCommand(Command commandToWait, Command commandToRun) {
        this.commandToWait = commandToWait;
        this.commandToRun = commandToRun;
    }

    @Override
    public void init() {

    }

    @Override
    public void run() {

    }



}
