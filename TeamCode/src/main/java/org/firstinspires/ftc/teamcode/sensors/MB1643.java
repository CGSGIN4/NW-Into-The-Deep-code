package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.utils.wall;


@I2cDeviceType
@DeviceProperties(
        name = "MB1643",
        description = "ultrasonic distance sensor",
        xmlTag = "MB1643"
)
public class MB1643 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {

    public wall wall1;
    public wall wall2;
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "MB1643 I2C Ultrasonic Distance Sensor";
    }

    public MB1643(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x70));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void ping() {
        deviceClient.write(TypeConversion.intToByteArray(0x51));
    }


    private short readRawRange(){
        return TypeConversion.byteArrayToShort(deviceClient.read( 2));
    }

    /**
     * Allow 100ms between pinging. Returns in centimeters
     */
    @Override
    public double getDistance(DistanceUnit unit){
        return unit.fromCm(readRawRange());
    }
}