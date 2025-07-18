/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.resources;

public class MathPrettyPrinterResource implements ResourcesContribution {
    @Override
    public void addToRegistry(ResourcesService registry) {
        registry.addResource("math-pretty-printer.jar", "math-pretty-printer/math-pretty-printer.jar");
    }
}
