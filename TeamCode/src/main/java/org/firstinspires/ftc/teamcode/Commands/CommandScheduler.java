package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class CommandScheduler {
    public enum GamepadInput{
        A_BUTTON,
        B_BUTTON,
        X_BUTTON,
        Y_BUTTON,
        BACK_BUTTON,
        START_BUTTON,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        DPAD_UP,
        DPAD_RIGHT,
        DPAD_DOWN,
        DPAD_LEFT,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
        LEFT_STICK_Y,
        LEFT_STICK_X,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
        RIGHT_STICK_Y,
        RIGHT_STICK_X
    }
    public enum GamepadIndex{
        PRIMARY,
        SECONDARY
    }


    private static CommandScheduler instance;

    private Telemetry telemetry;

    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Gamepad lastGamepad1;
    private Gamepad lastGamepad2;
    private Set<Command> scheduledCommands = new LinkedHashSet<>();
    private LinkedHashMap<Subsystem, Command> activeSubsystems = new LinkedHashMap<>();

    private LinkedHashMap<GamepadInput, Trigger> gamepad1Triggers = new LinkedHashMap<>();
    private LinkedHashMap<GamepadInput, Trigger> gamepad2Triggers = new LinkedHashMap<>();

    private Set<Command> defaultCommands = new LinkedHashSet<>();
    private Set<Command> activeCommands = new LinkedHashSet<>();

    public static synchronized CommandScheduler getInstance() {
        if (instance == null) {instance = new CommandScheduler();}
        return instance;
    }

    public void init(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();

    }
    public Trigger getTrigger(GamepadInput input, GamepadIndex index){
        if (index == GamepadIndex.PRIMARY){
            if (!gamepad1Triggers.containsKey(input)){
                gamepad1Triggers.put(input, new Trigger());
            }
            return gamepad1Triggers.get(input);
        }
        else {
            if (!gamepad2Triggers.containsKey(input)){
                gamepad2Triggers.put(input, new Trigger());
            }
            return gamepad2Triggers.get(input);
        }
    }
    public InputState getGamepadInput(GamepadInput input, GamepadIndex index){
        Gamepad gamepad = index == GamepadIndex.PRIMARY ? gamepad1 : gamepad2;
        Gamepad lastGamepad = index == GamepadIndex.PRIMARY ? lastGamepad1 : lastGamepad2;

        switch (input){
            case A_BUTTON:
                return new InputState(gamepad.a, lastGamepad.a);
            case B_BUTTON:
                return new InputState(gamepad.b, lastGamepad.b);
            case X_BUTTON:
                return new InputState(gamepad.x, lastGamepad.x);
            case Y_BUTTON:
                return new InputState(gamepad.y, lastGamepad.y);
            case BACK_BUTTON:
                return new InputState(gamepad.back, lastGamepad.back);
            case START_BUTTON:
                return new InputState(gamepad.start, lastGamepad.start);
            case LEFT_BUMPER:
                return new InputState(gamepad.left_bumper, lastGamepad.left_bumper);
            case RIGHT_BUMPER:
                return new InputState(gamepad.right_bumper, lastGamepad.right_bumper);
            case DPAD_UP:
                return new InputState(gamepad.dpad_up, lastGamepad.dpad_up);
            case DPAD_RIGHT:
                return new InputState(gamepad.dpad_right, lastGamepad.dpad_right);
            case DPAD_DOWN:
                return new InputState(gamepad.dpad_down, lastGamepad.dpad_down);
            case DPAD_LEFT:
                return new InputState(gamepad.dpad_left, lastGamepad.dpad_left);
            case LEFT_STICK_BUTTON:
                return new InputState(gamepad.left_stick_button, lastGamepad.left_stick_button);
            case RIGHT_STICK_BUTTON:
                return new InputState(gamepad.right_stick_button, lastGamepad.right_stick_button);
            case LEFT_STICK_Y:
                return new InputState(gamepad.left_stick_y, lastGamepad.left_stick_y);
            case LEFT_STICK_X:
                return new InputState(gamepad.left_stick_x, lastGamepad.left_stick_x);
            case LEFT_TRIGGER:
                return new InputState(gamepad.left_trigger, lastGamepad.left_trigger);
            case RIGHT_TRIGGER:
                return new InputState(gamepad.right_trigger, lastGamepad.right_trigger);
            case RIGHT_STICK_Y:
                return new InputState(gamepad.right_stick_y, lastGamepad.right_stick_y);
            case RIGHT_STICK_X:
                return new InputState(gamepad.right_stick_y, lastGamepad.right_stick_x);
        }
    return new InputState(0, 0);
    }
    public void schedule(Command... commands) {
        scheduledCommands.addAll(Arrays.asList(commands));

    }
    public void setDefaultCommands(Command... commands ){
        for (Command command:commands) {
            defaultCommands.add(command.setInterruptable(true));
        }
    }

    public void run() {
        if (scheduledCommands.isEmpty() && activeSubsystems.isEmpty() && defaultCommands.isEmpty() && gamepad1Triggers.isEmpty() && gamepad2Triggers.isEmpty()) {return;}
        this.telemetry.addData("after test of empty commands in CommandScheduler: ", "world");

        Iterator<Command> commandIterator = scheduledCommands.iterator();
        while (commandIterator.hasNext()) {
            Command command = commandIterator.next();
            Set<Command> conflicts = getActiveConflicts(command);
            Set<Subsystem> requirements = command.getRequirements();
            boolean conflicting = false;
            boolean willInterrupt = false;
            for (Command conflict : conflicts) {
                if (conflict.isInterruptable()) {
                    willInterrupt = true;
                } else {
                    conflicting = true;
                    break;
                }
            }

            if (!conflicting) {
                if (willInterrupt) {
                    for (Command conflict : conflicts) {
                        endCommand(conflict, true);
                    }
                }
                for (Subsystem requirement : requirements) {
                    activeSubsystems.put(requirement, command);
                }
                commandIterator.remove();
                activeCommands.add(command);
                command.init();
            }
        }

        for (Command defaultCommand:defaultCommands) {
            if (getActiveConflicts(defaultCommand).size()==0){
                for (Subsystem requirement : defaultCommand.getRequirements()) {
                    activeSubsystems.put(requirement, defaultCommand);
                }
                activeCommands.add(defaultCommand);
                defaultCommand.init();
            }
        }

        for (Command command : activeCommands) {
            command.run();
            if (command.isFinished()) {
                endCommand(command, false);
            }
        }
        for (GamepadInput input : gamepad1Triggers.keySet()){
            Trigger trigger = gamepad1Triggers.get(input);
            InputState inputState = getGamepadInput(input, GamepadIndex.PRIMARY);
            this.telemetry.addData("CommandScheduler gamepad input: ", input.name());
            this.telemetry.addData("CommandScheduler gamepad inputState: ", inputState.getValue());
            if (inputState.justPressed()){
                schedule(trigger.getOnJustPressed());
            }
            if (inputState.isPressed()){
                schedule(trigger.getOnPressed());
            }
            if (inputState.justReleased()){
                schedule(trigger.getOnJustReleased());
            }
            if (inputState.isReleased()){
                schedule(trigger.getOnReleased());
            }
        }
        for (GamepadInput input : gamepad2Triggers.keySet()){
            Trigger trigger = gamepad2Triggers.get(input);
            InputState inputState = getGamepadInput(input, GamepadIndex.SECONDARY);
            if (inputState.justPressed()){
                schedule(trigger.getOnJustPressed());
            }
            if (inputState.isPressed()){
                schedule(trigger.getOnPressed());
            }
            if (inputState.justReleased()){
                schedule(trigger.getOnJustReleased());
            }
            if (inputState.isReleased()){
                schedule(trigger.getOnReleased());
            }
        }
        gamepad1.copy(lastGamepad1);
        gamepad2.copy(lastGamepad2);
    }



    private Set<Command> getActiveConflicts(Command command) {
        Set<Command> conflictingCommands = new HashSet<>();
        Set<Subsystem> requirements = command.getRequirements();
        for (Subsystem requirement : activeSubsystems.keySet()) {
            if (requirements.contains(requirement)) {
                conflictingCommands.add(activeSubsystems.get(requirement));
            }
        }
        return conflictingCommands;
    }

    private void endCommand(Command command, boolean interrupted) {
        if (!activeCommands.contains(command)) return;

        Set<Subsystem> requirements = command.getRequirements();

        command.stop(interrupted);
        activeCommands.remove(command);
        for (Subsystem requirement : requirements) {
            activeSubsystems.remove(requirement);
        }
    }

    public void endAll() {
        scheduledCommands.clear();
        for (Command command : activeCommands) {
            endCommand(command, true);
        }
        gamepad1Triggers.clear();
        gamepad2Triggers.clear();
        defaultCommands.clear();
    }

    public void updateTelemetry() {
        telemetry.addData("Scheduled commands", getCommandNames(scheduledCommands));
        telemetry.addData("Active commands", getCommandNames(activeCommands));
        telemetry.addData("Active subsystems && commands", getSubsystemCommands(activeSubsystems));
    }

    private String getCommandNames(Set<Command> commands) {
        StringBuilder listOfNames = new StringBuilder();
        for (Command command : commands) {
            listOfNames.append(command.getName());
            listOfNames.append(", ");
        }
        listOfNames.append(";  ");
        return listOfNames.toString();
    }

    private String getSubsystemCommands(LinkedHashMap<Subsystem, Command> subsystems) {
        StringBuilder listOfNames = new StringBuilder();
        for (Map.Entry<Subsystem, Command> subsystemSet : subsystems.entrySet()) {
            listOfNames.append(subsystemSet.getKey().getName());
            listOfNames.append(": ");
            listOfNames.append(subsystemSet.getValue().getName());
            listOfNames.append(", ");
        }
        listOfNames.append("-;- ");
        return listOfNames.toString();
    }

}
