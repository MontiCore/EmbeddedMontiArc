package de.monticore.lang.monticar.visualization.emam.url;

import java.net.MalformedURLException;

public interface URLContribution {
    void addToRegistry(URLService registry) throws MalformedURLException;
}
