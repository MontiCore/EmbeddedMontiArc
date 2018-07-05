package de.monticore.lang.monticar.visualization.emam.url;

import java.net.URL;

public interface URLService {
    void addURL(String id, URL url);
    URL getURL(String id);
}
