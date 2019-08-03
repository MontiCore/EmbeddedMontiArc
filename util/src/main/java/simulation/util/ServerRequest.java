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
package simulation.util;

import org.apache.http.NameValuePair;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.CloseableHttpResponse;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.apache.http.message.BasicNameValuePair;
import org.jfree.util.Log;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

public class ServerRequest {

    public static String sendChargingStationRequest(String uri, String mapName, long osmID){
        CloseableHttpClient client = HttpClients.createDefault();
        HttpPost req = new HttpPost(uri);
        try {
            List<NameValuePair> form = new ArrayList<NameValuePair>();
            form.add(new BasicNameValuePair("mapName", mapName));
            form.add(new BasicNameValuePair("node", String.valueOf(osmID)));
            req.setEntity(new UrlEncodedFormEntity(form));

            CloseableHttpResponse resp = client.execute(req);
            int status = resp.getStatusLine().getStatusCode();
            if (status != 200) {
                org.jfree.util.Log.warn("Request error with status " + status);
                return "";
            }

            InputStream in = resp.getEntity().getContent();
            byte[] buffer = new byte[in.available()];
            in.read(buffer);
            return new String(buffer);
        } catch (Exception e) {
            Log.warn(e);
            e.printStackTrace();
        }
        return "";
    }
}
