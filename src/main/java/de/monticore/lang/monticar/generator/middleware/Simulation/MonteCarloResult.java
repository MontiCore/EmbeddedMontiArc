package de.monticore.lang.monticar.generator.middleware.Simulation;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.se_rwth.commons.logging.Log;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


public class MonteCarloResult {
    private EMAComponentInstanceSymbol component;
    private static int iterations;
    private static int numberOfClustersMC;

    public MonteCarloResult(EMAComponentInstanceSymbol component, int iterations, int numberOfClustersMC) {
        this.component = component;
        this.iterations = iterations;
        this.numberOfClustersMC = numberOfClustersMC;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return component;
    }

    public int getNumberOfClustersMC() {
        return numberOfClustersMC;
    }

    public int getIterations() {
        return iterations;
    }

    public void saveAsJson(String path, String filename) {
        JsonParser parser = new JsonParser();
        File dir = new File(path);
        File file = new File(dir, filename);
        JsonArray jsonArray = new JsonArray();
        if (file.exists() && !file.isDirectory()) {
            try {
                jsonArray = (JsonArray) parser.parse(new FileReader(file));
                JsonObject result = createJsonResultObject();
                jsonArray.add(result);
            } catch (IOException e) {
                Log.warn("Could not open clustering result file " + filename);
            }
        } else {
            JsonObject result = createJsonResultObject();
            jsonArray.add(result);
        }
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
        result.addProperty("MaxValueMC", MonteCarloIntegration.getMaxValue());
        result.addProperty("MinValueMC", MonteCarloIntegration.getMinValue());
        for (int i = 0; i < this.getIterations(); i++) {
            result.addProperty("MCResult(" + (i + 1) + ")", MonteCarloIntegration.getAverages()[i]);
        }
        result.addProperty("MCAverageResult", MonteCarloIntegration.getAverages()[this.getIterations()-1]);

        return result;
    }

}

