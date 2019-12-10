package connectivity.fri.sdk.example.lbrTorqueSineOverlay;

import java.util.Arrays;
import java.util.logging.Logger;

import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;
import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRState;

/**
 * Test client that superposes joint torques of the robot with sine waves.
 */
public class LBRTorqueSineOverlayClient extends LBRClient
{
    //!< bit mask encoding of joint torques to be overlaid
    private int _jointTorqueMask;

    //!< sine frequency (Hertz)
    private double _freqHz;

    //!< sine amplitude (Nm) of commanded torques
    private double _torqueAmpl;

    //!< current phase
    private double _phi;

    //!< phase step width
    private double _stepWidth;

    //!< commanded superposed torques
    private double[] _torques;

    /**
     * Constructor.
     * 
     * @param jointTorqueMask
     *            bit mask that encodes the joint torque indices to be overlaid
     *            by sine waves
     * @param freqHz
     *            Sine frequency in Hertz
     * @param torqueAmplitude
     *            Sine amplitude in Nm
     */
    public LBRTorqueSineOverlayClient(int jointTorqueMask, double freqHz,
            double torqueAmplitude)
    {
        _freqHz = freqHz;
        _torqueAmpl = torqueAmplitude;

        _jointTorqueMask = jointTorqueMask;

        _torques = new double[LBRState.NUMBER_OF_JOINTS];

        Logger.getAnonymousLogger().info("LBRTorqueSineOverlayClient initialized:\n"
                + "\tjoint torque mask: 0x" + Integer.toHexString(_jointTorqueMask) + "\n"
                + "\tfrequency (Hz): " + _freqHz + "\n"
                + "\tamplitude (Nm): " + _torqueAmpl + "\n");
    }

    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState)
    {
        switch (newState)
        {
        case MONITORING_READY:
            // (re)initialize sine parameters when entering Monitoring
            Arrays.fill(_torques, 0.0);
            _phi = 0.0;
            _stepWidth = 2 * Math.PI * _freqHz * getRobotState().getSampleTime();
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

        // If we want to command torques, we have to command them all the time; even in
        // waitForCommand(). This has to be done due to consistency checks. In this state it is 
        // only necessary, that some torque values are sent. The LBR does not take the 
        // specific value into account.
        if (getRobotState().getClientCommandMode() == ClientCommandMode.TORQUE)
        {
            getRobotCommand().setTorque(_torques);
        }
    }

    /**
     * Callback for the FRI state 'Commanding Active'.
     */
    @Override
    public void command()
    {

        // In command(), the joint values have to be sent. Which is done by calling
        // the base method.
        super.command();

        // Check for correct ClientCommandMode.
        if (getRobotState().getClientCommandMode() == ClientCommandMode.TORQUE)
        {
            double offset = _torqueAmpl * Math.sin(_phi);
            _phi += _stepWidth;

            if (_phi >= 2 * Math.PI)
            {
                _phi -= 2 * Math.PI;
            }

            for (int i = 0; i < LBRState.NUMBER_OF_JOINTS; i++)
            {
                if ((_jointTorqueMask & (1 << i)) != 0)
                {
                    _torques[i] = offset;
                }
            }
            // Set superposed joint torques.
            getRobotCommand().setTorque(_torques);
        }
    }
}
