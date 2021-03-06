package org.xero1425.base.motors;

import java.util.Map;
import java.util.HashMap;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.SettingsValue.SettingsType;

/// \file
/// This file contains the implementation of the MotorFactory.
/// The motor factory is used to create and initialize motors for all subsystems
/// based on data stored in the parameters file.
///

/// \brief A motor factory for the robot.
/// The motor factory creates motors based on entries in the settings file.  The
/// returned motors are high level motor classes (MotorController) that hides the
/// actual type of motor being used.  Groups of motors that operate in parallel, 
/// like multiple motors on a single side of the drivebase, are returned as a single
/// MotorController object that manages the motors as a group.
///
public class MotorFactory {
    private MessageLogger logger_;
    private ISettingsSupplier settings_;
    private Map<Integer, XeroMotorController> motors_;

    /// \brief This method creates a new motor factory.
    /// \param logger the message logger for the robot
    /// \param settings the settings file for the robot
    public MotorFactory(MessageLogger logger, ISettingsSupplier settings) {
        logger_ = logger;
        settings_ = settings;
        motors_ = new HashMap<Integer, XeroMotorController>();
    }

    /// \brief This method creates a new motor based on the settings in the settings
    /// file.
    ///
    /// \param name the base name of the motor
    /// \param id the ID of the motor in the settings file
    public XeroMotorController createMotor(String name, String id) {
        XeroMotorController ret = null;

        try {
            ret = createSingleMotor(name, id);
            if (ret != null)
                return ret;

            XeroMotorController.NeutralMode groupmode = getNeutralMode(id);
            boolean groupinverted = isInverted(id);
            boolean leaderinverted = false;
            int currentIndex = 1;
            MotorGroupController group = new MotorGroupController(logger_, name);
            ret = group;

            while (true) {
                String motorid = id + ":" + Integer.toString(currentIndex);
                XeroMotorController single = createSingleMotor(name + ":" + Integer.toString(currentIndex), motorid);
                if (single != null) {
                    //
                    // See if there is an inverted settings for this motor
                    //
                    boolean v = single.getInverted();

                    if (currentIndex == 1) {
                        //
                        // This is the first motor in the group. It is the leader. All of the other
                        // motors will follow this motor.
                        //
                        leaderinverted = v;

                        //
                        // If the group is inverted, invert the motor from its default setting
                        //
                        if (groupinverted)
                            v = !v;

                        //
                        // Set the motor to its proper inverted state
                        //
                        single.setInverted(v);

                    } else {
                        //
                        // If the leader is inverted, invert this motor relative to the
                        // inverter
                        //
                        if (leaderinverted)
                            v = !v;

                        if (groupinverted)
                            v = !v;

                        single.setInverted(v);

                        if (!leaderinverted && groupinverted)
                            v = !v;
                    }

                    if (groupmode != null) {
                        single.setNeutralMode(groupmode);
                    }

                    group.addMotor(single, v);

                    currentIndex++;
                } else {
                    if (currentIndex == 1) {
                        errorMessage(id, "no motors found that match this id");
                        return null;
                    }
                    break;
                }
            }
        } catch (Exception ex) {
            ret = null;
        }

        return ret;
    }

    private void errorMessage(String id, String msg) {
        logger_.startMessage(MessageType.Error);
        logger_.add("error creating motor '");
        logger_.add(id);
        logger_.add("' - ").add(msg);
        logger_.endMessage();
    }

    private XeroMotorController createSingleMotor(String name, String id)
            throws BadParameterTypeException, BadMotorRequestException, MotorRequestFailedException {
        String idparam = id + ":type";
        String canparam = id + ":canid";

        boolean hasid = settings_.isDefined(canparam) && settings_.getOrNull(canparam).isInteger();
        boolean hastype = settings_.isDefined(idparam) && settings_.getOrNull(idparam).isString();

        if (hastype && !hasid) {
            errorMessage(id, "missing motor id, cannot create motor");
            return null;
        }

        if (hasid && !hastype) {
            errorMessage(id, "missing motor type, cannot create motor");
            return null;
        }

        if (!hasid || !hastype)
            return null;

        int canid = settings_.getOrNull(canparam).getInteger();
        if (motors_.containsKey(canid)) {
            XeroMotorController dup = motors_.get(canid);
            errorMessage(id, "cannot create motor, can id is already in use '" + dup.getName() + "'");
            return null;
        }

        String type = settings_.getOrNull(idparam).getString();
        XeroMotorController ctrl = null;

        if (type.equals("romi")) {
            ctrl = new RomiMotorController(logger_, name, canid);
        } else if (type.equals("talon_srx")) {
            ctrl = new CTREMotorController(logger_, name, canid, CTREMotorController.MotorType.TalonSRX);
        } else if (type.equals("talon_fx")) {
            ctrl = new TalonFXMotorController(logger_, name, canid);
        } else if (type.equals("victor_spx")) {
            ctrl = new CTREMotorController(logger_, name, canid, CTREMotorController.MotorType.VictorSPX);
        } else if (type.equals("sparkmax_brushless")) {
            ctrl = new SparkMaxMotorController(logger_, name, canid, true);
        } else if (type.equals("sparmmax_brushed")) {
            ctrl = new SparkMaxMotorController(logger_, name, canid, false);
        } else {
            errorMessage(id, "motor type '" + type + "' is not a valid motor type");
            return null;
        }

        ctrl.setInverted(isInverted(id));
        XeroMotorController.NeutralMode nm = getNeutralMode(id);
        if (nm != null)
            ctrl.setNeutralMode(nm);

        // if (ctrl.hasPosition()) {
        //     ctrl.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent);
        // }

        boolean done = false ;
        while (!done)
        {
            try {
                Thread.sleep(25);
                done = true ;
            } catch (InterruptedException e) {
            }
        }

        return ctrl ;
    }

    private XeroMotorController.NeutralMode getNeutralMode(String id) throws BadParameterTypeException {
        SettingsValue v ;
        String pname = id + ":neutral_mode" ;
        
        v = settings_.getOrNull(pname) ;
        if (v == null)
            return null ;

        if (!v.isString()) {
            logger_.startMessage(MessageType.Error).add("parameter '").add(pname).add("'") ;
            logger_.add(" - does not have boolean type") ;
            throw new BadParameterTypeException(SettingsType.String, v.getType()) ;
        }

        XeroMotorController.NeutralMode mode ;
        if (v.getString().equals("brake"))
        {
            mode = XeroMotorController.NeutralMode.Brake ;
        }
        else if (v.getString().equals("coast"))
        {
            mode = XeroMotorController.NeutralMode.Coast ;
        }
        else 
        {
            logger_.startMessage(MessageType.Warning).add("parameter '").add(pname).add("'") ;
            logger_.add(" - is string but is not 'brake' or 'coast'") ;
            mode = null ;
        }

        return mode ;
    }

    private boolean isInverted(String id) throws BadParameterTypeException {
        SettingsValue v ;
        String pname = id + ":inverted" ;
        
        v = settings_.getOrNull(pname) ;
        if (v == null)
            return false ;

        if (!v.isBoolean()) {
            logger_.startMessage(MessageType.Error).add("parameter '").add(pname).add("'") ;
            logger_.add(" - does not have boolean type") ;
            throw new BadParameterTypeException(SettingsType.Boolean, v.getType()) ;
        }

        return v.getBoolean() ;
    }

} ;
