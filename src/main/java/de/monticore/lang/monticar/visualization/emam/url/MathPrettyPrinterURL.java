package de.monticore.lang.monticar.visualization.emam.url;

import java.net.MalformedURLException;
import java.net.URL;

public class MathPrettyPrinterURL implements URLContribution {
    @Override
    public void addToRegistry(URLService registry) throws MalformedURLException {
        registry.addURL("math-pretty-printer.zip", new URL("https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fshared%2F18.07.20.math-pretty-printer&files=math-pretty-printer.zip"));
    }
}
