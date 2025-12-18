package org.firstinspires.ftc.teamcode.Commands;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class TriggerList {

    public static TriggerList instance;

    private LinkedHashMap<BooleanSupplier, Trigger> commandTriggers = new LinkedHashMap<>();
    private LinkedHashMap<BooleanSupplier, Trigger> lastCommandTriggers = new LinkedHashMap<>();
    private LinkedHashMap<BooleanSupplier, Trigger> commandsToAdd = new LinkedHashMap<>();
    private LinkedHashMap<BooleanSupplier, Trigger> commandsToRemove = new LinkedHashMap<>();

    private boolean running = false;

    public static synchronized TriggerList getInstance() {
        if (instance == null) {
            instance = new TriggerList();
        }
        return instance;
    }

    public LinkedHashMap<BooleanSupplier, Trigger> getTriggers() {
        return commandTriggers;
    }

    public LinkedHashMap<BooleanSupplier, Trigger> getLastTriggers() {
        return lastCommandTriggers;
    }

    public boolean getLastState(BooleanSupplier key) {
        for (BooleanSupplier condition : lastCommandTriggers.keySet()) {
            if (condition == key) {
                return condition.getAsBoolean();
            }
        }
        return false;
    }

    public Trigger addTrigger(BooleanSupplier condition) {
        if (running) {
            if (!commandsToAdd.containsKey(condition)) {
                commandsToAdd.put(condition, new Trigger());
            }
            return commandsToAdd.get(condition);
        }
        if (!commandTriggers.containsKey(condition)) {
            commandTriggers.put(condition, new Trigger());
        }
        return commandTriggers.get(condition);
    }

    public void removeTrigger(BooleanSupplier condition) {
        if (running) {
            commandsToRemove.put(condition, new Trigger());
        } else {
            commandTriggers.remove(condition);
        }
    }

    public void updateList() {
        if (Collections.disjoint(commandTriggers.keySet(), commandsToAdd.keySet())) {
            commandTriggers.putAll(commandsToAdd);
        } else {
            for (BooleanSupplier condition : commandsToAdd.keySet()) {
                addTrigger(condition);
            }
        }
        for (BooleanSupplier condition : commandsToRemove.keySet()) {
            commandTriggers.remove(condition);
        }
        commandsToAdd.clear();
        commandsToRemove.clear();
    }

    public void copyTriggerList() {
        lastCommandTriggers = commandTriggers;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    public String getActiveTriggers() {
        StringBuilder listOfNames = new StringBuilder();
        for (Map.Entry<BooleanSupplier, Trigger> triggerSet : commandTriggers.entrySet()) {
            listOfNames.append(triggerSet.getKey().toString());
            listOfNames.append(": ");
            listOfNames.append(triggerSet.getValue().toString());
            listOfNames.append(", ");
        }
        return listOfNames.toString();
    }

    public void stop() {
        commandTriggers.clear();
        lastCommandTriggers.clear();
        commandsToAdd.clear();
        commandsToRemove.clear();
        running = false;
    }

}
