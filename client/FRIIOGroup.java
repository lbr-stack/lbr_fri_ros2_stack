package com.kuka.connectivity.fri.example;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>FRI</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class FRIIOGroup extends AbstractIOGroup
{
    /**
     * Constructor to create an instance of class 'FRI'.<br>
     * <i>This constructor is automatically generated. Please, do not modify!</i>
     *
     * @param controller
     *            the controller, which has access to the I/O group 'FRI'
     */
    @Inject
    public FRIIOGroup(Controller controller)
    {
        super(controller, "FRI");

        addInput("In_Bool_Clock_Enabled", IOTypes.BOOLEAN, 1);
        addDigitalOutput("Out_Bool_Enable_Clock", IOTypes.BOOLEAN, 1);
        addDigitalOutput("Out_Integer_Seconds", IOTypes.UNSIGNED_INTEGER, 32);
        addAnalogOutput("Out_Analog_Deci_Seconds", IOTypes.ANALOG, 32, 0.000000, 1.000000);
    }

    /**
     * Gets the value of the <b>digital input '<i>In_Bool_Clock_Enabled</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital input
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [false; true]
     *
     * @return current value of the digital input 'In_Bool_Clock_Enabled'
     */
    public boolean getIn_Bool_Clock_Enabled()
    {
        return getBooleanIOValue("In_Bool_Clock_Enabled", false);
    }

    /**
     * Gets the value of the <b>digital output '<i>Out_Bool_Enable_Clock</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [false; true]
     *
     * @return current value of the digital output 'Out_Bool_Enable_Clock'
     */
    public boolean getOut_Bool_Enable_Clock()
    {
        return getBooleanIOValue("Out_Bool_Enable_Clock", true);
    }

    /**
     * Sets the value of the <b>digital output '<i>Out_Bool_Enable_Clock</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [false; true]
     *
     * @param value
     *            the value, which has to be written to the digital output 'Out_Bool_Enable_Clock'
     */
    public void setOut_Bool_Enable_Clock(java.lang.Boolean value)
    {
        setDigitalOutput("Out_Bool_Enable_Clock", value);
    }

    /**
     * Gets the value of the <b>digital output '<i>Out_Integer_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0; 4294967295]
     *
     * @return current value of the digital output 'Out_Integer_Seconds'
     */
    public java.lang.Long getOut_Integer_Seconds()
    {
        return getNumberIOValue("Out_Integer_Seconds", true).longValue();
    }

    /**
     * Sets the value of the <b>digital output '<i>Out_Integer_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0; 4294967295]
     *
     * @param value
     *            the value, which has to be written to the digital output 'Out_Integer_Seconds'
     */
    public void setOut_Integer_Seconds(java.lang.Long value)
    {
        setDigitalOutput("Out_Integer_Seconds", value);
    }

    /**
     * Gets the value of the <b>analog output '<i>Out_Analog_Deci_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * analog output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0.0; 1.0]
     *
     * @return current value of the analog output 'Out_Analog_Deci_Seconds'
     */
    public double getOut_Analog_Deci_Seconds()
    {
        return getAnalogIOValue("Out_Analog_Deci_Seconds", true);
    }

    /**
     * Sets the value of the <b>analog output '<i>Out_Analog_Deci_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * analog output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0.0; 1.0]
     *
     * @param value
     *            the value, which has to be written to the analog output 'Out_Analog_Deci_Seconds'
     */
    public void setOut_Analog_Deci_Seconds(double value)
    {
        setAnalogOutput("Out_Analog_Deci_Seconds", value);
    }

}
