/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.json;

public class JSONObject extends org.json.JSONObject {
    public void mergeInto(JSONObject target) {
        this.entrySet().forEach(entry -> target.put(entry.getKey(), entry.getValue()));
    }
}
