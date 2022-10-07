/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.height;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.environment.osm.ApproximateConverter;
import de.rwth.montisim.simulation.environment.osm.Coordinates;
import de.rwth.montisim.simulation.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This class reads a given SRTM height file and creates a matrix of height values from it. These are then
 * interpolated by bilinear interpolation to retrieve the height at a given point on the world.
 *
 * More information on reading the data is here: https://gis.stackexchange.com/questions/43743/extracting-elevation-from-hgt-file
 * Documentation of SRTM: https://dds.cr.usgs.gov/srtm/version2_1/Documentation/SRTM_Topo.pdf
 */
public class SRTMHeightGenerator implements HeightGenerator {
    private static final double SAMPLE_SIZE = 3.0; // SRTM3 sampled at 3 arc-seconds
    private static final double INVALID_HEIGHT_VALUE = -32768.0; // Value of non-valid data sample in the file
    private static final double SECONDS_PER_DEG = 3600; // 1 degree has 3600 arc-seconds
    private static final int NUM_SAMPLES = 1201; // Number of rows as well as number of columns in the file
    private static final double SRTM_RESOLUTION = SAMPLE_SIZE / SECONDS_PER_DEG;
    private static final String HEIGHT_DATA_FILE_NAME = "N50E006.hgt";
    private static final String HEIGHT_DATA_FILE_NAME_PATTERN = "[NS]([0-9]{2})[EW]([0-9]{3})"; // e.g.: N50E006.hgt

    private double[][] heightMap;
    private Coordinates minCoords;
    private Coordinates maxCoords;
    private ApproximateConverter longLatToMeterConverter;

    public SRTMHeightGenerator() {
        // Read in height map from file
        InputStream heightDataStream = getClass().getResourceAsStream("/" + HEIGHT_DATA_FILE_NAME);
        heightMap = getHeightMapFromFile(heightDataStream);
        try {
            heightDataStream.close();
        } catch (IOException ex) {
        }

        // Read min lat/long from file name
        // This is done by pattern, so it can be dynamically done when using different file names
        Pattern pattern = Pattern.compile(HEIGHT_DATA_FILE_NAME_PATTERN);
        Matcher matcher = pattern.matcher(HEIGHT_DATA_FILE_NAME);
        if (matcher.find()) {
            minCoords = new Coordinates(Integer.valueOf(matcher.group(1)), Integer.valueOf(matcher.group(2)));
        }

        // Determine max lat/long
        maxCoords = new Coordinates(getLongitudeFromColumn(NUM_SAMPLES - 1), getLatitudeFromRow(NUM_SAMPLES - 1));
    }

    @Override
    public void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        this.longLatToMeterConverter = longLatToMeterConverter;
    }

    // NOTE: We need to get longitude and latitude, otherwise getting the value from height matrix will fail
    @Override
    public double getGround(double longitude, double latitude) {
        int flooredColumn = getColumnInHeightMap(longitude, true);
        int ceiledColumn = getColumnInHeightMap(longitude, false);
        int flooredRow = getRowInHeightMap(latitude, true);
        int ceiledRow = getRowInHeightMap(latitude, false);

        if (flooredColumn == -1 || ceiledColumn == -1 || flooredRow == -1 || ceiledRow == -1) {
            // If one value is invalid, just return 0 as we can not return a valid height
            return 0.0;
        }

        double flooredLong = getLongitudeFromColumn(flooredColumn);
        double ceiledLong = getLongitudeFromColumn(ceiledColumn);
        double flooredLat = getLatitudeFromRow(flooredRow);
        double ceiledLat = getLatitudeFromRow(ceiledRow);

        // Bilinear interpolation (for more information see https://en.wikipedia.org/wiki/Bilinear_interpolation )
        double denominator = (ceiledLong - flooredLong) * (ceiledLat - flooredLat);
        double flooredFlooredFactor = ((ceiledLong - longitude) * (ceiledLat - latitude)) / denominator;
        double ceiledFlooredFactor = ((longitude - flooredLong) * (ceiledLat - latitude)) / denominator;
        double flooredCeiledFactor = ((ceiledLong - longitude) * (latitude - flooredLat)) / denominator;
        double ceiledCeiledFactor = ((longitude - flooredLong) * (latitude - flooredLat)) / denominator;
        return flooredFlooredFactor * heightMap[flooredRow][flooredColumn]
                + ceiledFlooredFactor * heightMap[ceiledRow][flooredColumn]
                + flooredCeiledFactor * heightMap[flooredRow][ceiledColumn]
                + ceiledCeiledFactor * heightMap[ceiledRow][ceiledColumn];
    }

    @Override
    public double[][] toHeightMap() {
        return heightMap;
    }

    @Override
    public Vec2 getHeightMapMinPoint() {
        return longLatToMeterConverter.coordinatesToMeters(minCoords);
    }

    @Override
    public Vec2 getHeightMapMaxPoint() {
        return longLatToMeterConverter.coordinatesToMeters(maxCoords);
    }

    @Override
    public double getHeightMapDeltaX() {
        double minX = getHeightMapMinPoint().x;
        double maxX = getHeightMapMaxPoint().x;

        return (maxX - minX) / NUM_SAMPLES;
    }

    @Override
    public double getHeightMapDeltaY() {
        double minY = getHeightMapMinPoint().y;
        double maxY = getHeightMapMaxPoint().y;

        return (maxY - minY) / NUM_SAMPLES;
    }

    private static double[][] getHeightMapFromFile(InputStream heightDataStream) {
        // Construct the array
        double[][] heightData = new double[NUM_SAMPLES][];
        for (int row = 0; row < NUM_SAMPLES; row++) {
            heightData[row] = new double[NUM_SAMPLES];
        }

        // Rows are filled backwards, because in the height file they are written northern most first,
        // but we want them from lowest to highest latitude
        for (int row = NUM_SAMPLES - 1; row >= 0; row--) {
            for (int column = 0; column < NUM_SAMPLES; column++) {
                double heightValue;
                try {
                    // Read next 2-byte-height-value (according to big-endian-format)
                    int msb = heightDataStream.read();
                    int lsb = heightDataStream.read();

                    // Construct height value
                    heightValue = (msb << Byte.SIZE) | lsb;
                } catch (IOException ex) {
                    Log.warning("Exception occurred while parsing height data file at row=" + row + ",column=" + column);
                    heightValue = 0.0;
                }

                if (heightValue == INVALID_HEIGHT_VALUE) {
                    // TODO: Handle invalid values differently than just setting them to 0?
                    heightValue = 0.0;
                }

                heightData[row][column] = heightValue;
            }
        }

        return heightData;
    }

    private static int getColumnInHeightMap(double longitude, boolean useFloor) {
        long longDeg = (long) longitude;
        if (longDeg != 6) { // For now only allow longitude degree of 6, because only one file is deployed
            // TODO: Remove this and adapt to multiple height files
            return -1;
        }

        double unroundedColumn = (longitude - longDeg) / SRTM_RESOLUTION;

        return (int) (useFloor ? Math.floor(unroundedColumn) : Math.ceil(unroundedColumn));
    }

    private static int getRowInHeightMap(double latitude, boolean useFloor) {
        long latDeg = (long) latitude;
        if (latDeg != 50) { // For now only allow latitude degree of 50, because only one file is deployed
            // TODO: Remove this and adapt to multiple height files
            return -1;
        }
        double unroundedRow = (latitude - latDeg) / SRTM_RESOLUTION;

        return (int) (useFloor ? Math.floor(unroundedRow) : Math.ceil(unroundedRow));
    }

    private double getLongitudeFromColumn(long column) {
        return column * SRTM_RESOLUTION + minCoords.getLongitude();
    }

    private double getLatitudeFromRow(long row) {
        return row * SRTM_RESOLUTION + minCoords.getLatitude();
    }
}
