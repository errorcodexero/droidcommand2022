package org.xero1425.simulator.models;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class FMSModel extends SimulationModel {
    
    private enum FMSState {
        Initializing,
        Start,
        Test,
        BetweenTestAuto,
        Auto,
        BetweenAutoTeleop,
        Teleop,
        Closing,
        Done,
    } ;

    private FMSState state_ ;
    private double period_start_time_ ;
    private double start_time_  ;
    private double auto_time_ ;
    private double teleop_time_ ;
    private double between_time_ ;
    private double test_time_ ;
    private double closing_time_ ;

    public FMSModel(final SimulationEngine engine, final String model, final String inst) {
        super(engine, model, inst);

        start_time_ = 0.0 ;
        auto_time_ = 0.0 ;
        teleop_time_ = 0.0 ;
        between_time_ = 0.0 ;
        test_time_ = 0.0 ;
        closing_time_ = 0.0 ;        
    }

    public boolean create() {
        state_ = FMSState.Initializing ;

        if (hasProperty("autonomous")) {
            auto_time_ = setProperty("autonomous", getProperty("autonomous"), auto_time_) ;
        }
        else if (hasProperty("start")) {
            start_time_ = setProperty("start", getProperty("start"), start_time_) ;
        }        
        else if (hasProperty("between")) {
            between_time_ = setProperty("between", getProperty("between"), between_time_) ;
        }     
        else if (hasProperty("teleop")) {
            teleop_time_ = setProperty("teleop", getProperty("teleop"), teleop_time_) ;
        }
        else if (hasProperty("test")) {
            teleop_time_ = setProperty("test", getProperty("test"), test_time_) ;
        }

        setCreated();
        return true;
    }

    public void run(final double dt) {
        final double elapsed = getRobotTime() - period_start_time_ ;
        switch(state_)
        {
            case Initializing:
                DriverStationSim.setTest(false);
                DriverStationSim.setAutonomous(false);
                DriverStationSim.setEnabled(false);
                period_start_time_ = getRobotTime() ;
                state_ = FMSState.Start ;
                break ;

            case Start:
                if (elapsed >= start_time_)
                {
                    if (test_time_ > 0.0) {
                        DriverStationSim.setTest(true);
                        DriverStationSim.setAutonomous(false);
                        DriverStationSim.setEnabled(true);
                        state_ = FMSState.Test ;
                    } else {
                        if (auto_time_ > 0.0) {
                            DriverStationSim.setAutonomous(true);
                            DriverStationSim.setTest(false);
                            DriverStationSim.setEnabled(true);
                            state_ = FMSState.Auto ;                        
                        }
                        else {
                            DriverStationSim.setAutonomous(false);
                            DriverStationSim.setTest(false);
                            DriverStationSim.setEnabled(true);
                            state_ = FMSState.Teleop ;                            
                        }
                    }
                    period_start_time_ = getRobotTime() ;
                }
                break ;

            case Test:
                if (elapsed >= test_time_) 
                {
                    DriverStationSim.setTest(false);
                    DriverStationSim.setEnabled(false);
                    state_ = FMSState.BetweenTestAuto ;
                    period_start_time_ = getRobotTime() ;
                }
                break ;

            case BetweenTestAuto:
                if (elapsed >= between_time_) 
                {
                    if (auto_time_ > 0.0) {
                        DriverStationSim.setAutonomous(true);
                        DriverStationSim.setTest(false);
                        DriverStationSim.setEnabled(true);
                        state_ = FMSState.Auto ;
                    } else {
                        DriverStationSim.setAutonomous(false);
                        DriverStationSim.setTest(false);
                        DriverStationSim.setEnabled(true);
                        state_ = FMSState.Teleop ;                             
                    }
                    period_start_time_ = getRobotTime() ;
                }
                break ;

            case Auto:
                if (elapsed >= auto_time_)
                {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setEnabled(false);
                    state_ = FMSState.BetweenAutoTeleop ;
                    period_start_time_ = getRobotTime() ;
                }            
                break ;

            case BetweenAutoTeleop:
                if (elapsed >= between_time_)
                {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);
                    DriverStationSim.setEnabled(true);
                    state_ = FMSState.Teleop ;
                    period_start_time_ = getRobotTime() ;
                }              
                break ;

            case Teleop:
                if (elapsed >= teleop_time_)
                {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);                    
                    DriverStationSim.setEnabled(false);
                    state_ = FMSState.Closing ;
                    period_start_time_ = getRobotTime() ;
                }               
                break ;

            case Closing:
                if (elapsed >= closing_time_)
                {
                    state_ = FMSState.Done ;
                    period_start_time_ = getRobotTime() ;

                    getEngine().exitSimulator();
                }               
                break ;            

            case Done:
                break ;                                              
        }        
    }

    public boolean processEvent(final String name, final SettingsValue value) {
        boolean ret = false;

        if (name.equals("autonomous")) {
            auto_time_ = setProperty(name, value, auto_time_) ;
            ret = true ;
        }
        else if (name.equals("start")) {
            start_time_ = setProperty(name, value, start_time_) ;
            ret = true ;
        }
        else if (name.equals("between")) {
            between_time_ = setProperty(name, value, between_time_) ;
            ret = true ;
        }
        else if (name.equals("teleop")) {
            teleop_time_ = setProperty(name, value, teleop_time_) ;
            ret = true ;
        }
        else if (name.equals("test")) {
            test_time_ = setProperty(name, value, test_time_) ;
            ret = true ;
        }
        else if (name.equals("fms")) {
            if (!value.isBoolean()) {
                final MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a boolean").endMessage();
            }
            else {
                try {
                    DriverStationSim.setFmsAttached(value.getBoolean());
                } catch (final BadParameterTypeException e) {
                }
            }
        }

        return ret ;
    }

    private double setProperty(final String name, final SettingsValue v, double ret) {
        if (!v.isDouble()) {
            final MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" event name ").addQuoted(name);
            logger.add(" value is not a double").endMessage();
        }
        else {
            try {
                ret = v.getDouble();
            } catch (final BadParameterTypeException e) {
            }
        }

        return ret ;
    }

}