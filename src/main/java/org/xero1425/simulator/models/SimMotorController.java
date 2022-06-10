package org.xero1425.simulator.models;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.base.motors.CTREMotorController;
import org.xero1425.base.motors.XeroMotorController;
import org.xero1425.base.motors.SparkMaxMotorController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.SettingsValue;

public class SimMotorController {
    
    String name_ ;
    int handle_ ;
    private SimulationModel model_ ;
    private int count_ ;
    
    public SimMotorController(SimulationModel model, String name) {
        model_ = model;
        name_ = name ;
    }

    public int getCount() {
        return count_ ;
    }

    public double getPower() {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(handle_, XeroMotorController.SimPowerParamName) ;
        HALValue v = SimDeviceJNI.getSimValue(vhandle) ;
        if (v.getType() != HALValue.kDouble)
            return 0.0 ;

        return v.getDouble() ;
    }

    public void setEncoder(double v) {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(handle_, XeroMotorController.SimEncoderParamName) ;
        SimDeviceJNI.setSimValueDouble(vhandle, v);
    }

    public boolean usesTicks() {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(handle_, XeroMotorController.SimEncoderStoresTicksParamName) ;
        return SimDeviceJNI.getSimValue(vhandle).getBoolean() ;
    }

    public boolean createMotor() {
        handle_ = -1 ;
        if (createSingleMotor(name_ + ":motor")) {
            count_ = 1;
            return true;
        }

        int i = 1;
        while (true) {
            String mname = name_ + ":motor:" + i;
            if (!createSingleMotor(mname)) {
                if (i == 0)
                    return false;

                count_ = i;
                return true;
            }

            i++;
        }
    }

    private boolean createSingleMotor(String name) {
        int index = 0 ;

        if (!model_.hasProperty(name + ":index") || !model_.hasProperty(name + ":type"))
            return false;

        SettingsValue indexval = model_.getProperty(name + ":index");
        SettingsValue typeval = model_.getProperty(name + ":type");

        if (!indexval.isInteger())
            return false;

        if (!typeval.isString())
            return false;

        try {
            index = indexval.getInteger();
        } catch (BadParameterTypeException e) {
        }

        String t = null ;
        try {
            t = typeval.getString();
        } catch (BadParameterTypeException e) {
        }

        if (t.equals("talonfx") || t.equals("talonsrx")) {
            if (handle_ == -1)
            {
                handle_ = SimDeviceDataJNI.getSimDeviceHandle(CTREMotorController.SimDeviceName + "[" + index + "]") ;
            }
        }
        else if (t.equals("sparkmax-brushed")) {
            if (handle_ == -1)
                handle_ = SimDeviceDataJNI.getSimDeviceHandle(SparkMaxMotorController.SimDeviceNameBrushed + "[" + index + "]") ;
        }
        else if (t.equals("sparkmax-brushless")) {
            if (handle_ == -1)
                handle_ = SimDeviceDataJNI.getSimDeviceHandle(SparkMaxMotorController.SimDeviceNameBrushless + "[" + index + "]") ;            
        }        
        else {
            return false ;
        }

        if (handle_ == 0)
            return false ;
        
        return true ;        
    }

} ;