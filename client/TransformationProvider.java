package com.kuka.connectivity.fri.example;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

/**
 * Creates a FRI Session.
 */
public class TransformationProvider extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "172.31.1.148";

    }

    @Override
    public void run()
    {
        //Create some frames
        Frame objectBase = new Frame(World.Current.getRootFrame());
        Frame objectTip = new Frame(objectBase);
        objectTip.transform(Transformation.ofDeg(10.0, 10.0, 10.0, 45.0, 45.0, 45.0));

        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);

        // Send period from LBR to client
        friConfiguration.setSendPeriodMilliSec(10);

        // Send period multiply with integer gives the receive period from client to robot controller. 
        // In this example it is 30 milliseconds.
        friConfiguration.setReceiveMultiplier(3);

        // Select the frame from the scene graph whose transformation is changed by the client application.
        friConfiguration.registerTransformationProvider("PBase", objectBase);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // Output
        getLogger().info("Transformation from World of");
        for (int i = 0; i < 100; i++)
        {
            ThreadUtil.milliSleep(15);
            getLogger().info("Frame objectBase:\n" + objectBase.toStringInWorld());
            getLogger().info("Frame objectTip:\n" + objectTip.toStringInWorld());
        }

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final TransformationProvider app = new TransformationProvider();
        app.runApplication();
    }

}
