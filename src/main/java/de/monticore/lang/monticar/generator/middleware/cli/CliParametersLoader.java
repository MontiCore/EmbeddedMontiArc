package de.monticore.lang.monticar.generator.middleware.cli;

import alice.tuprolog.Int;
import com.google.gson.*;
import com.google.gson.stream.JsonReader;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.*;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.dynamic.*;
import de.se_rwth.commons.logging.Log;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

public class CliParametersLoader {
    private CliParametersLoader() {
    }


    public static CliParameters loadCliParameters(String filePath) throws FileNotFoundException {
        JsonReader jsonReader = new JsonReader(new FileReader(filePath));
        JsonDeserializer<DynamicParameter> deserializer = new JsonDeserializer<DynamicParameter>() {
            @Override
            public DynamicParameter deserialize(JsonElement jsonElement, Type type, JsonDeserializationContext jsonDeserializationContext) throws JsonParseException {
                if(jsonElement.isJsonObject()){
                    JsonObject jsonObj = (JsonObject) jsonElement;
                    Double min = jsonObj.has("min") ? jsonObj.get("min").getAsDouble() : null;
                    Double max = jsonObj.has("max") ? jsonObj.get("max").getAsDouble() : null;
                    Double step = jsonObj.has("step") ? jsonObj.get("step").getAsDouble() : null;
                    Integer count = jsonObj.has("count") ? jsonObj.get("count").getAsInt() : null;
                    return new GeneratorParamter(min,max,step,count);
                }else if(jsonElement.isJsonArray()) {
                    JsonArray jArray = (JsonArray) jsonElement;
                    List<Double> values = new ArrayList<>();
                    for(int i = 0; i < jArray.size(); i++){
                        values.add(jArray.get(i).getAsDouble());
                    }
                    return new ListParameter(values);
                }else{
                    return new ListParameter(jsonElement.getAsDouble());
                }
            }
        };

        Gson gson = new GsonBuilder()
                .registerTypeAdapter(DynamicAlgorithmCliParameters.class, new AlgorithmParametersInterfaceAdapter())
                .registerTypeAdapter(DynamicParameter.class, deserializer)
                .create();

        return gson.fromJson(jsonReader, CliParameters.class);
    }

    static final class AlgorithmParametersInterfaceAdapter implements JsonSerializer<DynamicAlgorithmCliParameters>, JsonDeserializer<DynamicAlgorithmCliParameters> {
        public JsonElement serialize(DynamicAlgorithmCliParameters object, Type interfaceType, JsonSerializationContext context) {
            return context.serialize(object);
        }

        public DynamicAlgorithmCliParameters deserialize(JsonElement elem, Type interfaceType, JsonDeserializationContext context) throws JsonParseException {
            final Type actualType = typeForName(((JsonObject)elem).get("name"));
            return context.deserialize(elem, actualType);
        }

        private Type typeForName(final JsonElement typeElem){
            String algoName = typeElem.getAsString().toLowerCase();
            switch (algoName){
                case "spectralclustering": return DynamicSpectralClusteringCliParameters.class;
                case "dbscan": return DynamicDBScanCliParameters.class;
                case "affinitypropagation": return DynamicAffinityPropagationCliParameters.class;
                case "markov": return DynamicMarkovCliParameters.class;
                default:{
                    Log.warn("Loaded config of unknown clustering algorithm: " + algoName);
                    return DynamicUnknownAlgorithmCliParameters.class;
                }
            }
        }
    }

}
