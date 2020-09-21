/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;

public class EMADynamicComponentSymbolReference extends EMAComponentSymbolReference {


    public EMADynamicComponentSymbolReference(final String name, final Scope definingScopeOfReference) {
        super(name, definingScopeOfReference);
    }


    public EMADynamicComponentSymbolReference(final String name, final Scope definingScopeOfReference, EmbeddedMontiArcDynamicSymbolTableCreator emastc) {
        super(name, definingScopeOfReference, emastc);
    }

    public EMADynamicComponentSymbol getReferencedSymbolAsDynamic(){
        return (EMADynamicComponentSymbol)getReferencedSymbol();
    }

    public Collection<EMADynamicEventHandlerSymbol> getEventHandlers(){
        return getReferencedSymbolAsDynamic().getEventHandlers();
    }

    public Collection<EMADynamicComponentInstantiationSymbol> getSubComponentsAsEMAD() {

        return this.reference.getReferencedSymbol().getSpannedScope().resolveLocally(EMADynamicComponentInstantiationSymbol.KIND);
    }

    @Override
    public void fixResolutions(EmbeddedMontiArcSymbolTableCreator emastc) {

        Collection<EMADynamicComponentInstantiationSymbol> instantiationSymbols = this.getSubComponentsAsEMAD();

        Collection<EMAPortArraySymbol> portArrays = this.getPortArrays();
        Collection<EMADynamicEventHandlerSymbol> eventHandlers = this.getEventHandlers();


        for (ResolutionDeclarationSymbol resDeclSym : getResolutionDeclarationSymbols()) {

            for(EMAPortArraySymbol pa : portArrays){
                pa.recreatePortArray(resDeclSym, emastc, this);
            }

            for(EMADynamicComponentInstantiationSymbol instantiationSymbol: instantiationSymbols){
                instantiationSymbol.fixResolutions(resDeclSym, emastc, this);
            }

            for(EMADynamicEventHandlerSymbol emaDynamicEventHandlerSymbol: eventHandlers){
                emaDynamicEventHandlerSymbol.fixCondition(resDeclSym);
                Log.debug(emaDynamicEventHandlerSymbol.condition.getTextualRepresentation(),"EMADynamicComponentSymbolReference");
            }
        }
    }

    @Override
    public Collection<EMAPortSymbol> getPortsList() {
        Collection<EMAPortSymbol> c = new ArrayList<>(super.getPortsList());
        c.addAll(this.getReferencedComponent().orElse(this).getSpannedScope().<EMADynamicPortArraySymbol>resolveLocally(EMADynamicPortArraySymbol.KIND));

        for(EMADynamicEventHandlerSymbol eh : this.getReferencedComponent().orElse(this).getSpannedScope().<EMADynamicEventHandlerSymbol>resolveLocally(EMADynamicEventHandlerSymbol.KIND)){
            ((ArrayList<EMAPortSymbol>) c).addAll(eh.getPorts());
        }

        return c;
    }
}
