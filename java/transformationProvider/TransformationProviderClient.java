package connectivity.fri.sdk.example.transformationProvider;

import com.kuka.connectivity.fastRobotInterface.clientSDK.base.TransformationClient;

/**
 * Example client for changing the transformation matrix of a robot frame.
 */
public class TransformationProviderClient extends TransformationClient
{
    private double[][] _transformationMatrix = {
            { 1, 0, 0, 10 },
            { 0, 1, 0, 10 },
            { 0, 0, 1, 10 }
    };

    @Override
    public void provide()
    {

        // Change the translational vector of the transformation matrix (all values in mm)
        double x = _transformationMatrix[0][3];
        if (x < 100)
        {
            x++;
        }
        else
        {
            x = 0;
        }
        _transformationMatrix[0][3] = x;

        double y = _transformationMatrix[1][3];
        if (y < 200)
        {
            y += 5;
        }
        else
        {
            y = 0;
        }
        _transformationMatrix[1][3] = y;

        double z = _transformationMatrix[2][3];
        if (z < 300)
        {
            z += 10;
        }
        else
        {
            z = 0;
        }
        _transformationMatrix[2][3] = z;

        // Set new transformation matrix for frame with identifier"PBase"
        setTransformation("PBase", _transformationMatrix, getTimeStampSec(), getTimeStampNanoSec());
    }
}
