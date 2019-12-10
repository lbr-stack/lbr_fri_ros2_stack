package connectivity.fri.sdk.example.lbrWrenchSineOverlay;

import java.util.logging.Logger;

import com.kuka.connectivity.fastRobotInterface.clientSDK.base.ClientApplication;
import com.kuka.connectivity.fastRobotInterface.clientSDK.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 * <p>
 * The application provides a {@link ClientApplication#connect}, a {@link ClientApplication#step()} and a
 * {@link ClientApplication#disconnect} method, which will be called successively in the application life-cycle.
 * 
 * 
 * @see ClientApplication#connect
 * @see ClientApplication#step()
 * @see ClientApplication#disconnect
 */
public class LBRWrenchSineOverlayApp
{
    private LBRWrenchSineOverlayApp()
    {
        // only for sonar
    }

    private static final int DEFAULT_PORTID = 30200;
    private static final double DEFAULT_FREQUENCY = 0.25;
    private static final double DEFAULT_AMPLITUDE = 5.0;

    /**
     * Main method.
     * 
     * @param argv
     *            the arguments
     */
    public static void main(String[] argv)
    {
        if (argv.length > 0)
        {
            if ("help".equals(argv[0]))
            {
                Logger.getAnonymousLogger().info("\nKUKA LBR wrench sine overlay test application\n\n\tCommand line arguments:");
                Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
                Logger.getAnonymousLogger().info("\t2) port ID (optional)");
                Logger.getAnonymousLogger().info("\t3) sine frequency in Hertz (for Fx) (optional)");
                Logger.getAnonymousLogger().info("\t4) sine amplitude in Newton (for Fx) (optional)");
                Logger.getAnonymousLogger().info("\t5) sine frequency in Hertz (for Fy) (optional)");
                Logger.getAnonymousLogger().info("\t6) sine amplitude in Newton (for Fy) (optional)");
                return;
            }
        }

        String hostname = (argv.length >= 1) ? argv[0] : null;
        int port = (argv.length >= 2) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;
        double frequencyX = (argv.length >= 3) ? Double.valueOf(argv[2]) : DEFAULT_FREQUENCY;
        double amplitudeX = (argv.length >= 4) ? Double.valueOf(argv[3]) : DEFAULT_AMPLITUDE;
        double frequencyY = (argv.length >= 5) ? Double.valueOf(argv[4]) : DEFAULT_FREQUENCY;
        double amplitudeY = (argv.length >= 6) ? Double.valueOf(argv[5]) : DEFAULT_AMPLITUDE;

        Logger.getAnonymousLogger().info("Enter LBRWrenchSineOverlay Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        // create new sine overlay client
        LBRWrenchSineOverlayClient client = new LBRWrenchSineOverlayClient(frequencyX, frequencyY, amplitudeX, amplitudeY);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Configuration */
        /*                                                                         */
        /***************************************************************************/

        // create new udp connection
        UdpConnection connection = new UdpConnection();

        // pass connection and client to a new FRI client application
        ClientApplication app = new ClientApplication(connection, client);

        // connect client application to KUKA Sunrise controller
        app.connect(port, hostname);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Execution mainloop */
        /*                                                                         */
        /***************************************************************************/

        // repeatedly call the step routine to receive and process FRI packets
        boolean success = true;
        while (success)
        {
            success = app.step();
        }

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Dispose */
        /*                                                                         */
        /***************************************************************************/

        // disconnect from controller
        app.disconnect();

        Logger.getAnonymousLogger().info("Exit LBRWrenchSineOverlay Client Application");
    }
}
