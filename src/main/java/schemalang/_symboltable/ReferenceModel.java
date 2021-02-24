package schemalang._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;

public class ReferenceModel {

    private EMAComponentSymbol emaComponent;

    public EMAComponentSymbol getEMAComponent() {
        return emaComponent;
    }

    public void setEMAComponent(EMAComponentSymbol emaComponent) {
        this.emaComponent = emaComponent;
    }
}