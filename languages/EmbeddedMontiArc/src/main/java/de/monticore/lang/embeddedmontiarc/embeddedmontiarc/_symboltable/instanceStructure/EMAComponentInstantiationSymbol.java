/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

/**
 * Represents an instance of a component.
 *
 */
public class EMAComponentInstantiationSymbol extends CommonScopeSpanningSymbol implements EMAElementInstanceSymbol {

    public static final EMAComponentInstantiationKind KIND = EMAComponentInstantiationKind.INSTANCE;

    private final EMAComponentSymbolReference componentType;

    /**
     * List of configuration arguments.
     */
    private List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs = new ArrayList<>();

    private String value = "";

    /**
     * Constructor for de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol
     *
     * @param name
     * @param componentType the referenced component definition
     */
    public EMAComponentInstantiationSymbol(String name, EMAComponentSymbolReference componentType) {
        super(name, KIND);
        this.componentType = componentType;

    }

    /**
     * @return componentType
     */
    public EMAComponentSymbolReference getComponentType() {
        return this.componentType;
    }

    /**
     * @return connectors of this component
     */
    public Collection<EMAConnectorSymbol> getSimpleConnectors() {
        return getSpannedScope().<EMAConnectorSymbol>resolveLocally(EMAConnectorSymbol.KIND);
    }

    public String getValue() {
        return value;
    }

    public void setValue(String value) {
        this.value = value;
    }

    /**
     * @return List of configuration arguments
     */
    public List<ValueSymbol<TypeReference<TypeSymbol>>> getConfigArguments() {
        return this.configArgs;
    }

    /**
     * @param cfg configuration arguments to add
     */
    public void addConfigArgument(ValueSymbol<TypeReference<TypeSymbol>> cfg) {
        this.configArgs.add(cfg);
    }

    /**
     * @param configArgs configuration arguments to set
     */
    public void setConfigArgs(List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs) {
        this.configArgs = configArgs;
    }

    @Override
    public String toString() {
        return SymbolPrinter.printEMAComponentInstantiation(this);
    }

    public Optional<InstanceInformation> getInstanceInformation(){
        for(InstanceInformation instanceInformation:InstancingRegister.instanceInformation){
            if(instanceInformation.getCompName().equals(getName())){
                return Optional.of(instanceInformation);
            }
        }
        return Optional.empty();
    }
}
