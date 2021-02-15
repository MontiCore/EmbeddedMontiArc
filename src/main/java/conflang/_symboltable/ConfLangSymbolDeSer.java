/* (c) https://github.com/MontiCore/monticore */

package conflang._symboltable;

import de.monticore.symboltable.serialization.json.JsonObject;

public class ConfLangSymbolDeSer extends ConfLangSymbolDeSerTOP {

    @Override
    protected void deserializeAddons(ConfLangSymbol symbol,
                                     JsonObject symbolJson) {
        String template = symbolJson.getStringMember("template");
        symbol.setTemplate(template);
    }
}
