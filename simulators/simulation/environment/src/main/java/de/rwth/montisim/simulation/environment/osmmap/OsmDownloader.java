/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.osmmap;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.net.URL;
import java.net.URLConnection;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;

public class OsmDownloader {
    private static final String OPENSTREETMAP_API_06 = "http://www.openstreetmap.org/api/0.6/";

    public char[] download(Coordinates center, double rangeInMeters) throws IOException {
        SimpleCoordinateConverter conv = new SimpleCoordinateConverter(center);
        Coordinates min_corner = new Coordinates();
        Coordinates max_corner = new Coordinates();
        conv.metersToCoords(new Vec2(rangeInMeters), min_corner);
        conv.metersToCoords(new Vec2(-rangeInMeters), max_corner);
        DecimalFormat format = new DecimalFormat("##0.0000000", DecimalFormatSymbols.getInstance(Locale.ENGLISH)); //$NON-NLS-1$
        String left = format.format(min_corner.lon);
        String bottom = format.format(min_corner.lat);
        String right = format.format(max_corner.lon);
        String top = format.format(max_corner.lat);

        URL osm = new URL(OPENSTREETMAP_API_06 + "map?bbox=" + left + "," + bottom + "," + right + "," + top);
        // HttpURLConnection connection = (HttpURLConnection) osm.openConnection();
        // connection.getInputStream().read(b);
        URLConnection con = osm.openConnection();
        Pattern p = Pattern.compile("text/html;\\s+charset=([^\\s]+)\\s*");
        Matcher m = p.matcher(con.getContentType());
        /* If Content-Type doesn't match this pre-conception, choose default and
         * hope for the best. */
        String charset = m.matches() ? m.group(1) : "ISO-8859-1";
        Reader r = new InputStreamReader(con.getInputStream(), charset);
        StringBuilder buf = new StringBuilder();
        while (true) {
            int ch = r.read();
            if (ch < 0)
                break;
            buf.append((char) ch);
        }
        char[] res = new char[buf.length()];
        buf.getChars(0, buf.length(), res, 0);
        return res;
    }
}