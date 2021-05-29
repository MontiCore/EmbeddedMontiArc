/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;

public interface CustomJson {
    void write(JsonWriter w, BuildContext context) throws SerializationException;
    void read(JsonTraverser t, ObjectIterable it, BuildContext context) throws SerializationException;
}
