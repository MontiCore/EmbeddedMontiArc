/* (c) https://github.com/MontiCore/monticore */
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
