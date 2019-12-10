package com.kuka.connectivity.fri.example;


import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fri.example.FRIIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class FRIIOApp extends RoboticsAPIApplication
{
    private String _clientName;
    
    @Inject
    private LBR lBR;
    
    @Inject
    private FRIIOGroup friGroup;
    
    @Override
    public void initialize()
    {
        _clientName = "172.31.1.148";
    }

    @Override
    public void run()
    {
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lBR, _clientName);
        friConfiguration.setSendPeriodMilliSec(5);

        friConfiguration.registerIO(friGroup.getInput("In_Bool_Clock_Enabled"));
        friConfiguration.registerIO(friGroup.getOutput("Out_Bool_Enable_Clock"));
        friConfiguration.registerIO(friGroup.getOutput("Out_Integer_Seconds"));
        friConfiguration.registerIO(friGroup.getOutput("Out_Analog_Deci_Seconds"));
        
        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(20, TimeUnit.SECONDS);
            getLogger().info("Connection to Client established");
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }
        
        getLogger().info("enable clock");
        ThreadUtil.milliSleep(5000);
        friGroup.setOut_Bool_Enable_Clock(true);
        
        getLogger().info("do something ...");
        ThreadUtil.milliSleep(10000);
        
        getLogger().info("disable clock");
        friGroup.setOut_Bool_Enable_Clock(false);
        
        getLogger().info("Close connection to client");
        friSession.close();

        
        lBR.move(ptpHome());
    }
}