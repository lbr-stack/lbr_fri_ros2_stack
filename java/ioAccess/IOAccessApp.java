package connectivity.fri.sdk.example.ioAccess;

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
public class IOAccessApp
{
    private IOAccessApp()
    {
        // only for sonar
    }

    private static final int DEFAULT_PORTID = 30200;

    /**
     * Main method.
     * 
     * @param argv
     *            command line arguments
     */
    public static void main(String[] argv)
    {
        if (argv.length > 0)
        {
            if ("help".equals(argv[0]))
            {
                Logger.getAnonymousLogger().info("\nKUKA Fieldbus access test application\n\n\tCommand line arguments:");
                Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
                Logger.getAnonymousLogger().info("\t2) port ID (optional)");
                return;
            }
        }

        // create new client
        IOAccessClient client = new IOAccessClient();
        Logger.getAnonymousLogger().info("Enter IOAccess Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        String hostname = (argv.length >= 1) ? argv[0] : null;
        int port = (argv.length >= 2) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;

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

        Logger.getAnonymousLogger().info("Exit IOAccess Client Application");
    }
}
