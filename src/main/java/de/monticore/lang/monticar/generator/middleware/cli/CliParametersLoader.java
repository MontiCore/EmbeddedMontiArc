package de.monticore.lang.monticar.generator.middleware.cli;

import com.google.gson.*;
import com.google.gson.stream.JsonReader;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.*;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.lang.reflect.Type;

public class CliParametersLoader {
    private CliParametersLoader() {
    }


    public static CliParameters loadCliParameters(String filePath) throws FileNotFoundException {
        JsonReader jsonReader = new JsonReader(new FileReader(filePath));
        Gson gson = new GsonBuilder().registerTypeAdapter(AlgorithmCliParameters.class, new AlgorithmParametersInterfaceAdapter()).create();
        return gson.fromJson(jsonReader, CliParameters.class);
    }

    static final class AlgorithmParametersInterfaceAdapter implements JsonSerializer<AlgorithmCliParameters>, JsonDeserializer<AlgorithmCliParameters> {
        public JsonElement serialize(AlgorithmCliParameters object, Type interfaceType, JsonSerializationContext context) {
            return context.serialize(object);
        }

        public AlgorithmCliParameters deserialize(JsonElement elem, Type interfaceType, JsonDeserializationContext context) throws JsonParseException {
            final Type actualType = typeForName(((JsonObject)elem).get("name"));
            return context.deserialize(elem, actualType);
        }

        private Type typeForName(final JsonElement typeElem){
            String algoName = typeElem.getAsString().toLowerCase();
            switch (algoName){
                case "spectralclustering": return SpectralClusteringCliParameters.class;
                case "dbscan": return DBScanCliParameters.class;
                case "affinitypropagation": return AffinityPropagationCliParameters.class;
                case "markov": return MarkovCliParameters.class;
                default: return UnknownAlgorithmCliParameters.class;
            }
        }
    }

}
