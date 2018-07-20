package de.monticore.lang.monticar.visualization.emam.url;

import java.net.MalformedURLException;
import java.net.URL;

public class VisualizationURL implements URLContribution {
    @Override
    public void addToRegistry(URLService registry) throws MalformedURLException {
        registry.addURL("visualization.zip", new URL("https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fshared%2F18.07.16.visualisation&files=visualisation.zip"));
    }
}
