package org.xero1425.base;

import java.net.NetworkInterface;
import java.util.Enumeration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.JsonSettingsParser;
import org.xero1425.misc.MessageDestination;
import org.xero1425.misc.MessageDestinationFile;
import org.xero1425.misc.MessageDestinationThumbFile;
import org.xero1425.misc.SimArgs;
import org.xero1425.base.motors.MotorFactory;

/// \file

/// \brief This is the base class for any XeroFramework robot.  It is derived from the
/// WPILib TimedRobot class and provides a complete infrastructure for robot code including
/// a settings file, a message logger, and a plotting system.  It is expected that a class will
/// be derived from this class that is specific to the robot being programmed.
public abstract class XeroRobot extends TimedRobot {

    // THe location of important file system paths used by the robot code
    private final RobotPaths robot_paths_ = new RobotPaths(isSimulation(), "") ;

    // The name of the robot
    private final String name_ ;

    // The amount of time between the current robot loop and the previous robot loop.  Should be
    // approximately 20 ms but may vary some.
    private double delta_time_ ;

    // The message logger for the robot
    private MessageLogger logger_ ;

    // The settings file supplier for the robot
    private ISettingsSupplier settings_ ;

    // The plot manager for the robot
    private PlotManager plot_mgr_ ;

    // The motor factor for creating new motors
    private MotorFactory motors_ ;

    // The MAC address for the ethernet controller
    private byte[] mac_addr_ ;

    // The LOGGER id for the XeroRobot class
    private int logger_id_ ;

    /// \brief The "subsystem" name for the message logger for this class
    public static final String LoggerName = "xerorobot" ;

    // A array to convert hex characters to integers
    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

    /// \brief Create a new XeroRobot robot
    /// \param period the robot loop timing (generally 20 ms)
    public XeroRobot(String name, final double period) {
        super(period);

        name_ = name ;

        // Setup the mesasge logger to log messages
        enableMessageLogger();
        logger_id_ = logger_.registerSubsystem(LoggerName) ;        
        logger_.startMessage(MessageType.Info).add("robot code starting").endMessage();

        if (RobotBase.isSimulation()) {
            String str = getSimulationFileName() ;
            if (str == null) {
                System.out.println("The code is setup to simulate, but the derived robot class did not provide a stimulus file") ;
                System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot") ;
            }
            else {
                SimulationEngine.initializeSimulator(this, logger_);
                addRobotSimulationModels() ;
                SimulationEngine.getInstance().initAll(str) ;
            }
        }

        // Get the network MAC address, used to determine comp bot versus practice bot
        getMacAddress();

        // Read the parameters file
        readParamsFile();

        // Enable messages in the message logger based on params file values
        enableMessagesFromSettingsFile() ;

        // Create the motor factor
        motors_ = new MotorFactory(logger_, settings_);

        // Create the plot manager
        plot_mgr_ = new PlotManager("/XeroPlot");
    }

    /// \brief Returns the message logger id for this class
    /// \returns the message logger id for this class
    public int getLoggerID() {
        return logger_id_ ;
    }

    /// \brief Returns the simulation stimulus file (JSON) name.  This method should be overridden by the 
    /// derived robot specific class.
    /// \returns the simulation stimulus file (JSON) name
    protected String getSimulationFileName() {
        return null ;
    }

    /// \brief Initialize the robot
    @Override
    public void robotInit() {
        boolean v;

        logger_.startMessage(MessageType.Info).add("initializing robot") ;
        if (DriverStation.isFMSAttached())
            logger_.add(" - FMS connected") ;
        else
            logger_.add(" - No FMS, practice mode") ;
        logger_.endMessage();

        /// Initialize the plotting subsystem
        try {
            v = settings_.get("system:plotting").getBoolean();
            if (v == true)
                plot_mgr_.enable(true);
            else
                plot_mgr_.enable(false);
        } catch (Exception ex) {
            //
            // Either the parameter is missing, or is not a boolean. In either
            // case we just turn off plotting
            plot_mgr_.enable(false);
        }
    }

    /// \brief Returns the current robot time in seconds
    /// \returns the current robot time in seconds
    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    /// \brief Returns the time between the last robot loop and the current robot loop
    /// \returns the time between the last robot loop and the current robot loop
    public double getDeltaTime() {
        return delta_time_;
    }

    /// \brief Returns the mesasge logger
    /// \returns the message logger
    public MessageLogger getMessageLogger() {
        return logger_;
    }

    /// \brief Returns the settings supplier
    /// \returns the setting supplier
    public ISettingsSupplier getSettingsSupplier() {
        return settings_;
    }

    /// \brief Returns the motor factory
    /// \returns the motor factory
    public MotorFactory getMotorFactory() {
        return motors_;
    }

    /// \brief Returns the mesasge logger
    /// \returns the message logger    
    public PlotManager getPlotManager() {
        return plot_mgr_;
    }

    /// \brief enable specific messages, epxected to be overridden by the derived class
    protected void enableMessages() {
    }

    /// \brief add specific models to the simulation, expected to be overridden by the derived class
    protected void addRobotSimulationModels() {
    }

    /// \brief return the name of the robot, expected to be overridden by the derived class
    public String getName() {
        return name_ ;
    }

    // \brief return the MAC address for the practice bot, expected to be overridden by the derived class
    /// \returns the MAC address for the practice bot
    protected byte[] getPracticeBotMacAddress() {
        return null;
    }

    /// \brief returns true if the current robot is the practice bot.  This is done by comparing the MAC
    /// address of the ethernet port on the RoboRio to a specific MAC address provided by the method getParcticeBotMacAddress().
    /// \returns true if the current robot is the practice bot
    protected boolean isPracticeBot() {
        if (mac_addr_ == null)
            return false;

        byte[] addr = getPracticeBotMacAddress();
        if (addr == null)
            return false;
        boolean ret = true ;
        for(int i = 0 ; i  < 6 ; i++)
        {
            byte b1 = addr[i] ;
            byte b2 = mac_addr_[i] ;
            if (b1 != b2)
                ret = false ;
        }
        return ret ;
    }

    private void enableMessagesFromSettingsFile() {
        String path = "system:messages" ;
        ISettingsSupplier p = getSettingsSupplier() ;
        MessageLogger m = getMessageLogger() ;

        var keys = p.getAllKeys(path) ;
        if (keys != null) {
            for(String key : keys)
            {
                try {
                    String longkey = path + ":" + key ;
                    SettingsValue v = p.get(longkey) ;
                    if (v.isBoolean() && v.getBoolean())
                    {
                        m.enableSubsystem(key) ;
                    }
                }
                catch(Exception ex)
                {
                }
            }
        }
    }

    private void enableMessageLogger() {
        String logfile = SimArgs.LogFileName ;
        MessageDestination dest ;

        logger_ = new MessageLogger();
        logger_.setTimeSource(new RobotTimeSource());

        if (logfile != null) {
            dest = new MessageDestinationFile(logfile) ;
        }
        else {
            dest = new MessageDestinationThumbFile(robot_paths_.logFileDirectory(), 250);
        }
        logger_.addDestination(dest);
        enableMessages();
    }

    private void readParamsFile() {
        JsonSettingsParser file = new JsonSettingsParser(logger_);

        String bot ;
        if (isPracticeBot())
            bot = "PRACTICE" ;
        else
            bot = "COMPETITION" ;

        file.addDefine(bot) ;
        logger_.startMessage(MessageType.Info).add("reading params for bot ").addQuoted(bot).endMessage() ;

        if (!file.readFile(robot_paths_.deployDirectory() + getName() + ".json")) {
            logger_.startMessage(MessageType.Error).add("error reading parameters file").endMessage();
        }

        settings_ = file ;
    }
     
    private void getMacAddress() {
        Enumeration<NetworkInterface> netlist ;
        mac_addr_ = null ;

        try {
            netlist = NetworkInterface.getNetworkInterfaces() ;
            while (netlist.hasMoreElements())
            {
                NetworkInterface ni = netlist.nextElement() ;
                String name = ni.getName() ;
                if (name.equals("lo"))
                    continue ;
                    
                mac_addr_ = ni.getHardwareAddress() ;
                break ;
            }
        }
        catch(Exception ex)
        {
            mac_addr_ = null ;
        }

        logger_.startMessage(MessageType.Info).add("Mac Address: ") ;
        if (mac_addr_ == null)
            logger_.add("NONE") ;
        else
        {
            for(int j = 0 ; j < mac_addr_.length ; j++)
            {
                int v = mac_addr_[j] & 0xFF;
                if (j != 0)
                    logger_.add(':') ;
                logger_.add(HEX_ARRAY[v >>> 4]) ;
                logger_.add(HEX_ARRAY[v & 0x0F]) ;
            }
        }
        logger_.endMessage();
    }
}
