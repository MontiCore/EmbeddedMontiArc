/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.serializer;

import com.google.gson.*;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralListAssignmentSymbol;

import java.lang.reflect.Type;

@Singleton
public class OptionSerializer implements JsonSerializer<OptionSymbol> {
    public static final String UNKNOWN = "unknown";

    @Override
    public JsonElement serialize(OptionSymbol option, Type type, JsonSerializationContext context) {
        return this.serialize(option);
    }

    protected JsonElement serialize(OptionSymbol option) {
        return option.getTypeSymbol().map(type -> this.serialize(option, type)).orElse(new JsonObject());
    }

    protected JsonElement serialize(OptionSymbol option, OptionTypeSymbol type) {
        JsonObject root = new JsonObject();
        JsonObject props = new JsonObject();

        root.add("props", props);
        root.addProperty("name", option.getName());
        root.addProperty("type", type.getFullName());

        option.getAssignmentSymbols().forEach(assignment -> props.add(assignment.getName(), this.serialize(assignment)));

        if (type.isComposite()) {
            JsonArray options = new JsonArray();

            props.add("options", options);
            option.getOptionSymbols().forEach(o -> options.add(this.serialize(o)));
        }

        return root;
    }

    protected JsonElement serialize(PropAssignmentSymbol assignment) {
        if (assignment.isLiteralAssignment()) return this.serialize((LiteralAssignmentSymbol)assignment);
        else if (assignment.isLiteralListAssignment()) return this.serialize((LiteralListAssignmentSymbol)assignment);
        else return new JsonPrimitive(UNKNOWN);
    }

    protected JsonElement serialize(LiteralAssignmentSymbol assignment) {
        if (assignment.isBoolean()) return new JsonPrimitive(assignment.getValueAsBoolean().orElse(false));
        else if (assignment.isString()) return new JsonPrimitive(assignment.getValueAsString().orElse(""));
        else if (assignment.isNumber()) return new JsonPrimitive(assignment.getValueAsNumber().orElse(0));
        else return new JsonPrimitive(UNKNOWN);
    }

    protected JsonElement serialize(LiteralListAssignmentSymbol assignment) {
        JsonArray array = new JsonArray();

        if (assignment.isBooleanList()) assignment.getValuesAsBooleans().forEach(array::add);
        else if (assignment.isStringList()) assignment.getValuesAsStrings().forEach(array::add);
        else if (assignment.isNumberList()) assignment.getValuesAsNumbers().forEach(array::add);

        return array;
    }
}
