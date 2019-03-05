package de.monticore.lang.monticar.generator.middleware.cli;

import com.google.gson.*;
import de.se_rwth.commons.logging.Log;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class StrictJsonDeserializer<T> implements JsonDeserializer<T> {

    private final List<String> optionalFieldNames;
    private final List<String> requiredFieldNames;
    private final Gson delegate;

    public StrictJsonDeserializer(List<String> optionalFieldNames, List<String> requiredFieldNames, Gson delegate) {
        this.optionalFieldNames = optionalFieldNames;
        this.requiredFieldNames = requiredFieldNames;
        this.delegate = delegate;
    }

    @Override
    public T deserialize(JsonElement jsonElement, Type type, JsonDeserializationContext jsonDeserializationContext) throws JsonParseException {
        if (!jsonElement.isJsonObject()) {
            Log.error("Expecting json object!");
        } else {
            JsonObject jsonObj = jsonElement.getAsJsonObject();
            Set<String> keys = jsonObj.keySet();

            checkRequiredFields(keys);
            checkOnlyValidFields(keys);
        }
        return delegate.fromJson(jsonElement, type);
    }

    private void checkOnlyValidFields(Set<String> keys) {
        List<String> allFieldNames = new ArrayList<>();
        allFieldNames.addAll(optionalFieldNames);
        allFieldNames.addAll(requiredFieldNames);

        for (String s : keys) {
            if (!allFieldNames.contains(s)) {
                Log.error("0x9DB6E: " + s + " is not a valid field name! Valid options: " + allFieldNames);
            }
        }
    }

    private void checkRequiredFields(Set<String> keys) {
        for (String requiredFieldName : requiredFieldNames) {
            if (!keys.contains(requiredFieldName)){
                Log.error("0x4C433: can not find required field " + requiredFieldName);
            }
        }
    }
}
