package de.monticore.lang.monticar.cnnarch.gluongenerator.decomposition;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public enum AllSubstitutes {
    RELU("Relu", "Activation"),
    SOFTMAX("Softmax", "softmax"),
    LOAD_NETWORK("LoadNetwork", "Reshape"),
    ADD("Add", "elemwise_add"),
    GLOBAL_POOLING("GlobalPooling", "Pooling", mapAttributes("global_pool", "True"));

    private final LayerSubstitute substitute;

    AllSubstitutes(String original, String... replacements) {
        substitute = new LayerSubstitute(original);
        for (String replacement : replacements) {
            substitute.addSubstitute(replacement);
        }
    }

    AllSubstitutes(String original, String replacement, Map<String, String> attributes) {
        substitute = new LayerSubstitute(original);
        substitute.addSubstitute(replacement);
        substitute.addAttribute(attributes);
    }

    private static Map<String, String> mapAttributes(String... attrs) {
        if (attrs.length % 2 != 0)
            throw new IllegalArgumentException("Missing value for attributes.");
        Map<String, String> map = new HashMap<>();
        for (int i = 0; i < attrs.length; i += 2) {
            map.put(attrs[i], attrs[i + 1]);
        }
        return map;
    }

    public LayerSubstitute getSubstitute() {
        return this.substitute;
    }

    public static List<LayerSubstitute> getAllSubstitutes() {
        List<LayerSubstitute> substitutes = new ArrayList<>();
        for (AllSubstitutes substitution : values()) {
            substitutes.add(substitution.getSubstitute());
        }
        return substitutes;
    }
}
