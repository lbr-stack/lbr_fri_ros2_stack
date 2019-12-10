package connectivity.fri.sdk.example.lbrWrenchSineOverlay;

import java.util.Arrays;
import java.util.logging.Logger;

import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;

/**
 * Test client that can add additional forces along the X and Y direction of the
 * tool center point of the LBR. The forces change their amplitude sinusoidal.
 */
public class LBRWrenchSineOverlayClient extends LBRClient
{
    //!< number of elements in a Cartesian vector
    private static final int CART_VECTOR_DIM = 6;

    //!< sine frequency x-direction (Hertz)
    private double _freqHzX;

    //!< sine frequency y-direction (Hertz)
    private double _freqHzY;

    //!< sine amplitude x-direction (Newton)
    private double _amplRadX;

    //!< sine amplitude y-direction (Newton)
    private double _amplRadY;

    //!< commanded wrench
    private double[] _wrench;

    //!< delta phi for sine in x-direction
    private double _deltaPhiX;

    //!< delta phi for sine in y-direction
    private double _deltaPhiY;

    //!< current phase for sine in x-direction
    private double _phiX;

    //!< current phase for sine in y-direction
    private double _phiY;

    /**
     * Constructor.
     * 
     * @param freqHzX
     *            sine frequency in Hertz of force in X-direction
     * @param freqHzY
     *            sine frequency in Hertz of force in Y-direction
     * @param amplRadX
     *            sine amplitude in radians of force in X-direction
     * @param amplRadY
     *            sine amplitude in radians of force in Y-direction
     */
    public LBRWrenchSineOverlayClient(double freqHzX, double freqHzY,
            double amplRadX, double amplRadY)
    {
        _freqHzX = freqHzX;
        _freqHzY = freqHzY;
        _amplRadX = amplRadX;
        _amplRadY = amplRadY;

        _wrench = new double[CART_VECTOR_DIM];

        Logger.getAnonymousLogger().info("LBRWrenchSineOverlayClient initialized:\n"
                + "\tfrequency (Hz):  X = " + freqHzX + ", Y = " + freqHzY + "\n"
                + "\tamplitude (N):   X = " + amplRadX + ", Y = " + amplRadY + "\n");
    }

    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState)
    {
        switch (newState)
        {
        case MONITORING_READY:
            // (re)initialize sine parameters when entering Monitoring
            Arrays.fill(_wrench, 0.0);
            _phiX = 0.0;
            _phiY = 0.0;
            _deltaPhiX = 2 * Math.PI * _freqHzX * getRobotState().getSampleTime();
            _deltaPhiY = 2 * Math.PI * _freqHzY * getRobotState().getSampleTime();
            break;
        case IDLE:
        case MONITORING_WAIT:
        default:
            break;
        }
    }

    @Override
    public void waitForCommand()
    {
        // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
        // the base method.
        super.waitForCommand();

        // If we want to command wrenches, we have to command them all the time; even in
        // waitForCommand(). This has to be done due to consistency checks.In this state it is
        // only necessary, that some wrench-values are sent. The LBR does not take the specific 
        // value into account.
        if (getRobotState().getClientCommandMode() == ClientCommandMode.WRENCH)
        {
            getRobotCommand().setWrench(_wrench);
        }
    }

    @Override
    public void command()
    {
        // In command(), the joint values have to be sent. Which is done by calling
        // the base method.
        super.command();

        // Check for correct ClientCommandMode.
        if (getRobotState().getClientCommandMode() == ClientCommandMode.WRENCH)
        {
            // Calculate new forces in x and y direction.
            _wrench[0] = _amplRadX * Math.sin(_phiX);
            _wrench[1] = _amplRadY * Math.sin(_phiY);

            _phiX += _deltaPhiX;
            _phiY += _deltaPhiY;

            if (_phiX >= 2 * Math.PI)
            {
                _phiX -= 2 * Math.PI;
            }
            if (_phiY >= 2 * Math.PI)
            {
                _phiY -= 2 * Math.PI;
            }
            // Set wrench vector.
            getRobotCommand().setWrench(_wrench);
        }
    }
}
