/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.model;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.util.Base64;

/**
 * A screenshot.
 */
public class Screenshot {

    private int carId;
    private BufferedImage screenshot;
    private String extension;

    public static Screenshot fromReceivedScreenshot(final ReceivedScreenshot s) throws IOException {
        assert s != null;
        final String[] parts = s.getScreenshot().split(",");
        assert parts.length == 2;
        final String prefix = parts[0];
        final String base64Image = parts[1];
        final String ext = Screenshot.parseExtension(prefix);
        final BufferedImage image = Screenshot.parseImage(base64Image);
        assert ext != null && ext.trim().length() > 0;
        assert image != null && image.getWidth() > 0 && image.getHeight() > 0;
        final Screenshot result = new Screenshot();
        result.setCarId(s.getCarId());
        result.setExtension(ext);
        result.setScreenshot(image);
        return result;
    }

    private static String parseExtension(String prefix) {
        assert prefix != null;
        prefix = prefix.trim().toLowerCase();
        final int i = prefix.indexOf('/');
        final int j = prefix.indexOf(';');
        assert i >= 0 && j >= 0 && j > i;
        return prefix.substring(i + 1, j);
    }

    private static BufferedImage parseImage(final String base64Image) throws IOException {
        final byte[] imageByte = Base64.getDecoder().decode(base64Image);
        try (ByteArrayInputStream bis = new ByteArrayInputStream(imageByte)) {
            return ImageIO.read(bis);
        }
    }

    public int getCarId() {
        return carId;
    }

    public void setCarId(int carId) {
        this.carId = carId;
    }

    public BufferedImage getScreenshot() {
        return screenshot;
    }

    public void setScreenshot(BufferedImage screenshot) {
        this.screenshot = screenshot;
    }

    public String getExtension() {
        return extension;
    }

    public void setExtension(String extension) {
        this.extension = extension;
    }

}
