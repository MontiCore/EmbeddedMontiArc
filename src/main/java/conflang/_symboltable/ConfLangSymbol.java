/* (c) https://github.com/MontiCore/monticore */
package conflang._symboltable;

import de.monticore.symboltable.ISymbol;

import java.util.Collection;
import java.util.Optional;

public class ConfLangSymbol extends ConfLangSymbolTOP {

    protected String template;

    public ConfLangSymbol(final String name) {
        super(name);
        template = "";
    }

    /**
     * @return template
     */
    public String getTemplate() {
        return this.template;
    }

    /**
     * @param template the template to set
     */
    public void setTemplate(String template) {
        this.template = template;
    }

}
