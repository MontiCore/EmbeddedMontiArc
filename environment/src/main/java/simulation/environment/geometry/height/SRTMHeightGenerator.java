/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.environment.geometry.height;

import simulation.util.Log;

import java.io.File;
import java.io.RandomAccessFile;
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
    private static final double SECONDS_PER_DEG = 3600;
    private static final int NUM_SAMPLES = 1201; // Number of rows as well as number of columns in the file
    private static final int NUM_BYTES_PER_HEIGHT_VALUE = 2; // Each height value is 2 bytes large
    private static final double SRTM_RESOLUTION = SAMPLE_SIZE / SECONDS_PER_DEG;
    private static final String HEIGHT_DATA_FILE_NAME_PATTERN = "[NS]([0-9]{2})[EW]([0-9]{3})"; // e.g.: N50E006.hgt

    private double[][] heightMap;
    private double minLatitude;
    private double minLongitude;
    private double maxLatitude;
    private double maxLongitude;

    public SRTMHeightGenerator(File heightDataFile) {
        // Read in height map from file
        heightMap = getHeightMapFromFile(heightDataFile);

        // Read min lat/long from file name
        String heightDataFileName = heightDataFile.getName();
        Pattern pattern = Pattern.compile(HEIGHT_DATA_FILE_NAME_PATTERN);
        Matcher matcher = pattern.matcher(heightDataFileName);
        if (matcher.find()) {
            minLatitude = Integer.valueOf(matcher.group(1));
            minLongitude = Integer.valueOf(matcher.group(2));
        }

        // Determine max lat/long
        maxLatitude = getLatitudeFromRow(NUM_SAMPLES - 1);
        maxLongitude = getLongitudeFromColumn(NUM_SAMPLES - 1);
    }

    // TODO: An Server schicken: Höhenmatrix, MinLat, MinLong, Delta, Max Werte
    // NOTE: We need to get longitude and latitude, otherwise getting the value from height matrix will fail
    @Override
    public double getGround(double longitude, double latitude) {
        int flooredColumn = getColumnInHeightMap(longitude, true);
        int ceiledColumn = getColumnInHeightMap(longitude, false);
        int flooredRow = getRowInHeightMap(latitude, true);
        int ceiledRow = getRowInHeightMap(latitude, false);

        double flooredLong = getLongitudeFromColumn(flooredColumn);
        double ceiledLong = getLongitudeFromColumn(ceiledColumn);
        double flooredLat = getLatitudeFromRow(flooredRow);
        double ceiledLat = getLatitudeFromRow(ceiledRow);

        // Bilinear interpolation (for more information see https://en.wikipedia.org/wiki/Bilinear_interpolation )
        double denominator = (ceiledLong - flooredLong)*(ceiledLat - flooredLat);
        double flooredFlooredFactor = ((ceiledLong - longitude)*(ceiledLat - latitude)) / denominator;
        double ceiledFlooredFactor = ((longitude - flooredLong)*(ceiledLat - latitude)) / denominator;
        double flooredCeiledFactor = ((ceiledLong - longitude)*(latitude - flooredLat)) / denominator;
        double ceiledCeiledFactor = ((longitude - flooredLong)*(latitude - flooredLat)) / denominator;
        return flooredFlooredFactor * heightMap[flooredRow][flooredColumn]
                + ceiledFlooredFactor * heightMap[ceiledRow][flooredColumn]
                + flooredCeiledFactor * heightMap[flooredRow][ceiledColumn]
                + ceiledCeiledFactor * heightMap[ceiledRow][ceiledColumn];
    }

    @Override
    public double[][] toHeightMap() {
        return heightMap;
    }

    private static double[][] getHeightMapFromFile(File heightDataFile) {
        double[][] heightData = new double[NUM_SAMPLES][];
        try(RandomAccessFile heightDataFileSeeker = new RandomAccessFile(heightDataFile, "r")) {
            for (int row = 0; row < NUM_SAMPLES; row++) {
                heightData[row] = new double[NUM_SAMPLES];
                for (int column = 0; column < NUM_SAMPLES; column++) {
                    // We invert the rows, because in the height file they are written northern most first,
                    // but we want them from lowest latitude to highest latitude
                    heightDataFileSeeker.seek(((NUM_SAMPLES - row - 1) * NUM_SAMPLES + column) * NUM_BYTES_PER_HEIGHT_VALUE);

                    // Read next 2-byte-height-value (according to big-endian-format)
                    int msb = heightDataFileSeeker.read();
                    int lsb = heightDataFileSeeker.read();

                    // Construct height value and assign in height map
                    double heightValue = (msb << Byte.SIZE) | lsb;
                    if (heightValue == INVALID_HEIGHT_VALUE) { // Check if height value is valid
                        // TODO: Handle invalid values better than just setting them to 0 maybe?
                        heightValue = 0.0;
                    }

                    heightData[row][column] = heightValue;
                }
            }
        }
        catch (Exception ex) {
            Log.warning("Exception occurred while parsing height data file!");
            ex.printStackTrace();
        }

        return heightData;
    }

    private static int getColumnInHeightMap(double longitude, boolean useFloor) {
        long longDeg = (long)longitude;
        double unroundedColumn = (longitude - longDeg) / SRTM_RESOLUTION;

        return (int)(useFloor ? Math.floor(unroundedColumn) : Math.ceil(unroundedColumn));
    }

    private static int getRowInHeightMap(double latitude, boolean useFloor) {
        long latDeg = (long)latitude;
        double unroundedRow = (latitude - latDeg) / SRTM_RESOLUTION;

        return (int)(useFloor ? Math.floor(unroundedRow) : Math.ceil(unroundedRow));
    }

    private double getLongitudeFromColumn(long column) {
        return column * SRTM_RESOLUTION + minLongitude;
    }

    private double getLatitudeFromRow(long row) {
        return row * SRTM_RESOLUTION + minLatitude;
    }
}
