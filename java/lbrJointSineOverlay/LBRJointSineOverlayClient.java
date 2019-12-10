package connectivity.fri.sdk.example.lbrJointSineOverlay;

import java.util.logging.Logger;

import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;
import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRState;

/**
 * Test client that can overlay interpolator joint positions with sine waves.
 */
public class LBRJointSineOverlayClient extends LBRClient
{
    //!< bit mask encoding of joints to be overlaid
    private int _jointMask;

    //!< sine frequency (Hertz)
    private double _freqHz;

    //!< sine amplitude (radians)
    private double _amplRad;

    //!< filter coefficient
    private double _filterCoeff;

    //!< offset for current interpolation step
    private double _offset;

    //!< current phase
    private double _phi;

    //!< phase step width
    private double _stepWidth;

    /**
     * Constructor.
     * 
     * @param jointMask
     *            bit mask that encodes the joint indices to be overlaid by sine
     *            waves
     * @param freqHz
     *            sine frequency in Hertz
     * @param amplRad
     *            sine amplitude in radians
     * @param filterCoeff
     *            filter coefficient between 0 (filter off) and 1 (max filter)
     */
    public LBRJointSineOverlayClient(int jointMask, double freqHz,
            double amplRad, double filterCoeff)
    {
        _freqHz = freqHz;
        _amplRad = amplRad;
        _filterCoeff = filterCoeff;

        _jointMask = jointMask;

        Logger.getAnonymousLogger().info("LBRJointSineOverlayClient initialized:\n"
                + "\tjoint mask: 0x" + Integer.toHexString(_jointMask) + "\n"
                + "\tfrequency (Hz): " + _freqHz + "\n"
                + "\tamplitude (rad): " + _amplRad + "\n"
                + "\tfilterCoeff: " + _filterCoeff + "\n");
    }

    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState)
    {
        switch (newState)
        {
        case MONITORING_READY:
            // (re)initialize sine parameters when entering Monitoring
            _offset = 0.0;
            _phi = 0.0;
            _stepWidth = 2 * Math.PI * _freqHz * getRobotState().getSampleTime();
            break;
        case IDLE:
        case MONITORING_WAIT:
        default:
            break;
        }
    }

    /**
     * Callback for the FRI state 'Commanding Active'.
     */
    @Override
    public void command()
    {
        // calculate new offset
        double newOffset = _amplRad * Math.sin(_phi);
        _offset = _offset * _filterCoeff + newOffset * (1.0 - _filterCoeff);
        _phi += _stepWidth;
        if (_phi >= 2 * Math.PI)
        {
            _phi -= 2 * Math.PI;
        }

        // add offset to ipo joint position for all masked joints
        double[] jointPos = getRobotState().getIpoJointPosition();

        for (int i = 0; i < LBRState.NUMBER_OF_JOINTS; i++)
        {
            if ((_jointMask & (1 << i)) != 0)
            {
                jointPos[i] += _offset;
            }
        }

        getRobotCommand().setJointPosition(jointPos);
    }
}
