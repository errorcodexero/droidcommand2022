package org.xero1425.base.motors ;

import java.lang.Exception ;

/// \file
/// This file contains the implementation of the BadMotorRequestException.  This
/// exception is thrown when some is requested of the motor class that cannot be
/// completed.
///

/// \brief An exception that is thrown when a bad request is made to a motor
public class BadMotorRequestException extends Exception
{  
    private XeroMotorController motor_ ;

    /// \brief The UID for the class for serialization
    static final long serialVersionUID = 42 ;

    /// \brief This method creates a new BadMotorRequestException
    /// \param motor the motor that causeed the exception
    /// \param msg a string describing the cause of the exception
    public BadMotorRequestException(XeroMotorController motor, String msg) {
        super("motor '" + motor.getName() + "' - " + msg) ;

        motor_ = motor ;
    }

    /// \brief Return the motor that caused the exception
    /// \returns the motor that caused the exception
    public XeroMotorController getMotor() {
        return motor_ ;
    }
}
