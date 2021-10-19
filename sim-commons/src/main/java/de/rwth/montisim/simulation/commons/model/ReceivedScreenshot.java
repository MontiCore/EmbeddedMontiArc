/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.model;

/**
 * Screenshot received from a client.
 */
public class ReceivedScreenshot {

    private int carId;
    private String screenshot;

    public int getCarId() {
        return carId;
    }

    public void setCarId(int carId) {
        this.carId = carId;
    }

    public String getScreenshot() {
        return screenshot;
    }

    public void setScreenshot(String screenshot) {
        this.screenshot = screenshot;
    }

}
