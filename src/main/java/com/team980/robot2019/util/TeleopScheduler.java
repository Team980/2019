package com.team980.robot2019.util;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

/**
 * It's kind of a joke that we even have this.
 * I discovered that it was possible to rumble the Xbox controller,
 * so I rigged this up so we could remind the drivers when it's time to climb.
 */
public class TeleopScheduler {

    private Timer timer;

    private ArrayList<Double> timeList;
    private ArrayList<Runnable> functionList;

    public TeleopScheduler() {
        timer = new Timer();

        timeList = new ArrayList<>();
        functionList = new ArrayList<>();
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }

    public void execute() {
        for (int i = 0; i < timeList.size(); i++) {
            if (timer.get() > timeList.get(i)) {
                functionList.get(i).run();

                timeList.remove(i);
                functionList.remove(i);
                break; //NOTE we assume one time = one function, which is safe
            }
        }
    }

    public void disable() {
        timer.stop();

        timeList.clear();
        functionList.clear();
    }

    public void queue(double time, Runnable function) {
        timeList.add(time);
        functionList.add(function);
    }
}
