/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.Simulation;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


public class MonteCarloResult {
    private EMAComponentInstanceSymbol component;
    private MonteCarloIntegration sim;

    public MonteCarloResult(EMAComponentInstanceSymbol component, MonteCarloIntegration sim) {
        this.component = component;
        this.sim = sim;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return component;
    }

    public int getNumberOfClustersMC() {
        return sim.getNumberOfClusters();
    }

    public int getIterations() {
        return sim.getIterations();
    }

    public void saveAsJson(String path, String filename) {
        JsonParser parser = new JsonParser();
        File dir = new File(path);
        File file = new File(dir, filename);
        JsonArray jsonArray = new JsonArray();
        JsonObject result = createJsonResultObject();
        jsonArray.add(result);

        try {
            file.getParentFile().mkdirs();
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(jsonArray.toString());
            fileWriter.flush();
        } catch (IOException io) {
            Log.warn("Could not write clustering results to json file " + filename);
        }
    }

    private JsonObject createJsonResultObject() {
        JsonObject result = new JsonObject();
        result.addProperty("Iterations", this.getIterations());
        result.addProperty("NumberOfClusters", this.getNumberOfClustersMC());
        result.addProperty("MaxValueMC", sim.getMaxValue());
        result.addProperty("MinValueMC", sim.getMinValue());
        for (int i = 0; i < this.getIterations(); i++) {
            result.addProperty("MCResult(" + (i + 1) + ")", sim.getAverages()[i]);
        }
        result.addProperty("MCAverageResult", sim.getAverages()[this.getIterations() - 1]);

        return result;
    }

}

