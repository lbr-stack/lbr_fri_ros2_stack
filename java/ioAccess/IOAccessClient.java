package connectivity.fri.sdk.example.ioAccess;

import java.util.concurrent.TimeUnit;

import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;

/**
 * Example client for Fieldbus access.
 */
public class IOAccessClient extends LBRClient
{
    private long _startTime;
    private final static double _analogMax = 1;
    private final static double _analogMin = 0;
    private final static long _digitalMax = 60;
    private final static long _digitalMin = 0;
    private final static double _step = 0.1;

    public IOAccessClient()
    {
        _startTime = System.nanoTime();
    }

    @Override
    public void onStateChange(FRISessionState newState, FRISessionState oldState)
    {
        switch (newState)
        {
        case MONITORING_WAIT:
        case MONITORING_READY:
        case COMMANDING_WAIT:
        case COMMANDING_ACTIVE:
        default:
        {
            break;
        }
        }
    }

    @Override
    public void monitor()
    {
        getAndSetExample();
    }

    @Override
    public void waitForCommand()
    {
        getAndSetExample();
    }

    @Override
    public void command()
    {
        getAndSetExample();
    }

    private void getAndSetExample()
    {
        // LIMIT
        double tempAnalog = getRobotState().getAnalogIOValue("FRI.Out_Analog_Deci_Seconds");
        if (tempAnalog > _analogMax)
        {
            getRobotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", _analogMax);
        }
        else if (tempAnalog < _analogMin)
        {
            getRobotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", _analogMin);
        }

        long tempDigital = getRobotState().getDigitalIOValue("FRI.Out_Integer_Seconds");
        if (tempDigital > _digitalMax)
        {
            getRobotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", _digitalMax);
        }
        else if (tempDigital < _digitalMin)
        {
            getRobotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", _digitalMin);
        }

        boolean isEnabled = getRobotState().getBooleanIOValue("FRI.In_Bool_Clock_Enabled");

        if (isEnabled)
        {
            long now = System.nanoTime();
            long difference = now - _startTime;
            int milliSecDiff = (int) (TimeUnit.NANOSECONDS.toMillis(difference));
            if (milliSecDiff >= 100)
            {
                double analogValue = getRobotState().getAnalogIOValue("FRI.Out_Analog_Deci_Seconds");
                analogValue = analogValue + _step;
                if (analogValue < _analogMax)
                {
                    getRobotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", analogValue);
                }
                else
                {
                    getRobotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", _analogMin);

                    long digitalValue = getRobotState().getDigitalIOValue("FRI.Out_Integer_Seconds") + 1;
                    if (digitalValue < _digitalMax)
                    {
                        getRobotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", digitalValue);
                    }
                    else
                    {
                        getRobotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", _digitalMin);
                    }
                }
                _startTime = now;
            }
        }
    }
}
