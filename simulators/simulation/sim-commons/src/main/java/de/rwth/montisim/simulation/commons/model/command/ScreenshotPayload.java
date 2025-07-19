/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.model.command;

/**
 * Data for taking a screenshot.
 */
public class ScreenshotPayload {

    public final static String CAMERA_CAR_FRONT = "carFront";

    private String cameraToUse;
    private int carId;

    public String getCameraToUse() {
        return cameraToUse;
    }

    public void setCameraToUse(String cameraToUse) {
        this.cameraToUse = cameraToUse;
    }

    public int getCarId() {
        return carId;
    }

    public void setCarId(int carId) {
        this.carId = carId;
    }

}
