package org.xero1425.simulator.engine;

import org.xero1425.misc.SettingsValue;

public class SimulationAssertEvent extends SimulationEvent {

    private String subsystem_;
    private String name_;
    private String setting_;
    private SettingsValue value_;
    private double tolerance_;

    public SimulationAssertEvent(double t, String subsystem, String name, SettingsValue v) {
        super(t);

        subsystem_ = subsystem;
        name_ = name;
        value_ = v;
        setting_ = null;

        tolerance_ = 1e-9;
    }

    public SimulationAssertEvent(double t, String subsystem, String name, String setting) {
        super(t);
        subsystem_ = subsystem;
        name_ = name;
        value_ = null;
        setting_ = setting;

        tolerance_ = 1e-9;
    }

    public void run(SimulationEngine engine) {
    }

    public String toString() {
        return "SimulationAssertEvent";
    }

    public void setTolerance(double v) {
        tolerance_ = v;
    }
}