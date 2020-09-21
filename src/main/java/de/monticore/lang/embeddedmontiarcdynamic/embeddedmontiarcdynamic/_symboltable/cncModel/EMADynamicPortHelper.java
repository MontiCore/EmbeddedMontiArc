/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantBoolean;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantSIUnit;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTArrayAccess;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTPort;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.literals.literals._ast.ASTBooleanLiteral;
import de.monticore.numberunit.Rationals;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.MutableScope;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.StringTransformations;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class EMADynamicPortHelper extends EMAPortHelper {

    //<editor-fold desc="Single Instance">
    protected static EMADynamicPortHelper INSTANCE = null;
    public static EMADynamicPortHelper getINSTANCE(){
        if(INSTANCE == null){
            EMADynamicPortHelper.init();
        }
        return INSTANCE;
    }
    protected void setINSTANCE(EMADynamicPortHelper builder){
        INSTANCE = builder;
    }
    public static EMADynamicPortHelper init(){
        EMADynamicPortHelper b = new EMADynamicPortHelper();
        b.setINSTANCE(b);
        return b;
    }
    //</editor-fold>

    //<editor-fold desc="New port">

    public EMAPortBuilder newEMAPortSymbolBUILDER(ASTPort node, String name, MCTypeReference<? extends MCTypeSymbol> typeRef) {
        return EMAPortSymbol.builder().setASTNode(Optional.of(node))
                .setName(name)
                .setTypeReference(typeRef)
                .setDirection(node.isIncoming())
                .setConfig(node.getAdaptableKeywordOpt().isPresent());
    }

    public EMAPortSymbol newEMAPortSymbol(ASTPort node, String name, MCTypeReference<? extends MCTypeSymbol> typeRef){
        return this.newEMAPortSymbolBUILDER(node, name, typeRef).build();
    }


    public String doUnitNumberResolution(Optional<ASTUnitNumberResolution> node, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        String name = null;
        if (node.isPresent()) {
            name = node.get().doResolution((symbolTableCreator.componentStack.peek()).getResolutionDeclarationSymbols());
        }

        return name;
    }

    public int getNumberIfIntLiteralIsPresent(Optional<ASTUnitNumberResolution> node){
        if(node.isPresent() && node.get().getNumberWithUnitOpt().isPresent()){
            if(node.get().getNumber().isPresent()) {
                return node.get().getNumber().get().intValue();
            }
        }
        return 0;
    }

    public boolean numberIsInfinite(Optional<ASTUnitNumberResolution> node){
        return node.isPresent() && node.get().getNumberWithUnitOpt().isPresent() && node.get().getNumberWithUnit().isPlusInfinite();
    }

    public EMAPortArraySymbol newEMAPortArraySymbol(ASTPort node, String name, MCTypeReference<? extends MCTypeSymbol> typeRef, EmbeddedMontiArcDynamicSymbolTableCreator dstc){

        String arraySizeDependsOn = doUnitNumberResolution(node.getUnitNumberResolutionOpt(), dstc);

        EMAPortArraySymbol portArraySymbol = new EMAPortArraySymbol(name, arraySizeDependsOn);

        portArraySymbol.setTypeReference(typeRef);
        portArraySymbol.setDirection(node.isIncoming());
        portArraySymbol.setConfig(node.getAdaptableKeywordOpt().isPresent());

        if(node.getUnitNumberResolutionOpt().isPresent() && node.getUnitNumberResolutionOpt().get().getNumberWithUnitOpt().isPresent()){
            EMAPortHelper.portCreationIntLiteralPresent(node, portArraySymbol, name, typeRef, dstc);
        }

        return portArraySymbol;
    }

    public EMADynamicPortArraySymbol newEMADynamicPortArraySymbol(ASTPort node, String name, MCTypeReference<? extends MCTypeSymbol> typeRef, EmbeddedMontiArcDynamicSymbolTableCreator dstc){

        String nonDynamicDimensionDependsOn = doUnitNumberResolution(node.getUnitNumberResolutionOpt(), dstc);
        String dimensionDependsOn = doUnitNumberResolution(node.getDynamicNumberOfPortsOpt(), dstc);

        EMADynamicPortArraySymbol portArraySymbol = new EMADynamicPortArraySymbol(name, nonDynamicDimensionDependsOn, dimensionDependsOn);

        portArraySymbol.setTypeReference(typeRef);
        portArraySymbol.setDirection(node.isIncoming());
        portArraySymbol.setDynamic(node.isDynamic());
        portArraySymbol.setConfig(node.getAdaptableKeywordOpt().isPresent());

        portArraySymbol.setNonDynamicDimension(this.getNumberIfIntLiteralIsPresent(node.getUnitNumberResolutionOpt()));


        if(numberIsInfinite(node.getDynamicNumberOfPortsOpt())){
            portArraySymbol.setDimensionInfinite(true);
            portArraySymbol.setDimension(portArraySymbol.getNonDynamicDimension());
        }else {
            portArraySymbol.setDimension(this.getNumberIfIntLiteralIsPresent(node.getDynamicNumberOfPortsOpt()));
        }

        return portArraySymbol;
    }

    //</editor-fold>

    //<editor-fold desc="Conector Setup">

    //<editor-fold desc="Constant Connector">


    public int countComponentArrayInstancesWithDynamic(String componentName, EmbeddedMontiArcSymbolTableCreator stc){

        if(stc == null){
            return 0;
        }

        MutableScope curScope = stc.currentScope().get();
        boolean present = true;
        int counter = 0;
        Log.debug("" + componentName, "RESOLVING");

        if(stc instanceof EmbeddedMontiArcDynamicSymbolTableCreator){
            Optional<EMADynamicComponentInstantiationSymbol> sym = curScope.resolve(componentName, EMADynamicComponentInstantiationSymbol.KIND);
            if(sym.isPresent()){
                if(sym.get().isDynamic()){
                    counter = sym.get().getNonDynamicDimension();
                }else if(sym.get().isArray()){
                    counter = sym.get().getDimension();
                }
            }
        }
        else {
            while (present) {
                present = curScope.resolve(componentName + "[" + (counter + 1) + "]", EMAComponentInstantiationSymbol.KIND).isPresent();
                if (present) {
                    ++counter;
                }
            }
        }
        return counter;
    }

    public List<String> getComponentNamePartsWithDynamic(ASTQualifiedNameWithArrayAndStar portNameStar,
                                                     EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        ASTQualifiedNameWithArray portName = (ASTQualifiedNameWithArray)portNameStar.getQualifiedNameWithArray();
        List<String> names = new ArrayList<String>();
        String name = "";
        if (portName.getCompNameOpt().isPresent()) {
            name += portName.getCompName();
            if (portName.getCompArrayOpt().isPresent()) {
                ASTArrayAccess arrayAccess = (ASTArrayAccess) portName.getCompArray();
                if (arrayAccess.getIntLiteralOpt().isPresent()) {
                    name += "[" + arrayAccess.getIntLiteral().getNumber().get().intValue()
                            + "]";
                    name += ".";
                    names.add(name);
                } else if (arrayAccess.getLowerboundOpt().isPresent()) {
//                    names = getmnCompNameParts(name, portName);
                    int lower = arrayAccess.getLowerbound().getNumber().get().intValue();
                    int upper = arrayAccess.getUpperbound().getNumber().get().intValue();
                    for (int i = lower; i <= upper; ++i) {
                        String instanceName = name;
                        instanceName += "[" + i + "]";
                        instanceName += ".";
                        names.add(instanceName);
                    }
                } else if(arrayAccess.isDynamicNewPort()) {
                    name += "[?].";
                    names.add(name);
                } else {
                    int size = this.countComponentArrayInstancesWithDynamic(name, symbolTableCreator);
                    for (int i = 1; i <= size; ++i) {
                        String instanceName = name;
                        instanceName += "[" + i + "].";
                        names.add(instanceName);
                    }
                }
            } else {
                names.add(name + ".");
            }
        } else {
            names.add("");
        }
        return names;
    }

    public List<String> getPortNamePartsWithDynamic(ASTQualifiedNameWithArray portName,
                                                EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        List<String> names = new ArrayList<String>();
        String name = "";

        name = portName.getPortName();
        if (portName.getPortArrayOpt().isPresent()) {
            ASTArrayAccess arrayAccess = (ASTArrayAccess) portName.getPortArray();

            if (arrayAccess.getIntLiteralOpt().isPresent()) {
                name += "["
                        + arrayAccess.getIntLiteral().getNumber().get().intValue()
                        + "]";
                names.add(name);
            } else if (arrayAccess.getLowerboundOpt().isPresent()) {
//                names = getmnPortNameParts(name, portName);
                int lower = ((Double)arrayAccess.getLowerbound().getNumber().get()).intValue();
                int upper = ((Double)arrayAccess.getUpperbound().getNumber().get()).intValue();

                for(int i = lower; i <= upper; ++i) {
                    String instanceName = name + "[" + i + "]";
                    names.add(instanceName);
                    Log.debug("Name:", "Added MNPortName");
                }

            } else if(arrayAccess.isDynamicNewPort()) {
                name += "[?]";
                names.add(name);
            } else {
                Log.debug(portName.toString(), "PortName:");

                int size = 1;
                if(symbolTableCreator != null){
                    size = countPortArrayInstances(name, portName.getCompNameOpt().orElse(""),
                            portName.getCompArrayOpt().orElse(null), symbolTableCreator);
                }

                Log.debug("Size" + size, "PortNameParts");
                for (int i = 1; i <= size; ++i) {
                    String instanceName = name;
                    instanceName += "[" + i + "]";
                    names.add(instanceName);
                }
            }
        } else {
            Log.debug("No PortArrayName was specified", "PortArray");
            names.add(name);
        }
        return names;
    }

    public List<String> getPortNameWithDynamic(ASTQualifiedNameWithArrayAndStar portName, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        List<String> names = new ArrayList<String>();

        List<String> compNameParts = getComponentNamePartsWithDynamic(portName, symbolTableCreator);

        List<String> portNameParts;
        portNameParts = getPortNamePartsWithDynamic(portName.getQualifiedNameWithArray(), symbolTableCreator);

        Log.debug("portName: " + portName.getQualifiedNameWithArray() + " " + compNameParts.size(), "CompNameParts");
        Log.debug("" + portNameParts.size(), "PortNameParts");
        for (String compNamePart : compNameParts) {
            for (String portNamePart : portNameParts) {
                String curName = compNamePart + portNamePart;
                names.add(curName);
            }
        }

        return names;

    }



    /**
     * converts a UnitNumberLiteral to EMAConstantValue for a constant Port
     */
    public EMAConstantSIUnit getConstantValue(ASTNumberWithUnit si_unit) {
        Unit unit = si_unit.getUnit();
        Rational rational = Rationals.doubleToRational(si_unit.getNumber().get());
        return new EMAConstantSIUnit(rational, unit);
    }

    /**
     * converts a BooleanLiteral to EMAConstantValue for a constant Port
     */
    public static EMAConstantBoolean getConstantValueBool(ASTBooleanLiteral astBooleanLiteral) {
        return new EMAConstantBoolean(astBooleanLiteral.getValue());
    }


    public void constantPortSetup(
            ASTConnector node,
            EmbeddedMontiArcDynamicSymbolTableCreator symbolTableCreator) {
        int counter = 0;

        EMAPortSymbol emaConstantPortSymbol = ConstantPortHelper.createConstantPortSymbol(node, symbolTableCreator); //createConstantPortSymbol(node, symbolTableCreator);
        symbolTableCreator.addToScope(emaConstantPortSymbol);

        // TODO: Remove Workaround!!!!!!!
        if(emaConstantPortSymbol.getConstantValue().isPresent() && emaConstantPortSymbol.getConstantValue().get() instanceof EMAConstantBoolean){
            emaConstantPortSymbol.setConstantValue(getConstantValueBool(node.getBoolLiteral()));
        }

        for (ASTQualifiedNameWithArrayAndStar target : node.getTargets().getQualifiedNameWithArrayAndStarList()) {
            counter = 0;

            List<String> targetNames = getPortNameWithDynamic(target, symbolTableCreator);


            for(String targetName : targetNames) {
                Log.debug("" + targetName, "target");

                EMADynamicConnectorSymbol sym = new EMADynamicConnectorSymbol(targetName);
                sym.setConstantEMAPortSymbol(emaConstantPortSymbol);
                sym.setSource(emaConstantPortSymbol.getName());
                sym.setTarget(targetName);

                setDynamicInConnectorSourceTarget(sym, null, target.getQualifiedNameWithArray());

                Log.debug(sym.getTarget(), "TARGETNAME SET TO");

                symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
            }
            ++counter;

        }
    }
    //</editor-fold>

    //<editor-fold desc="Non Constant Connector">

    public void nonConstantPortSetup(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector node,
                                     EmbeddedMontiArcDynamicSymbolTableCreator symbolTableCreator) {

        List<String> sourceNames = getPortNameWithDynamic(node.getSource(), symbolTableCreator);
        Log.info("" + sourceNames.size(), "SourcePorts");
        int counter = 0, targetnum = 0;
        for (ASTQualifiedNameWithArrayAndStar target : node.getTargets().getQualifiedNameWithArrayAndStarList()) {
            counter = 0;
            targetnum = 0;
            for (String sourceName : sourceNames) {
                List<String> targetNames = getPortNameWithDynamic(target, symbolTableCreator);
                targetnum = targetNames.size();
                if (counter < targetnum) {
                    String targetName = targetNames.get(counter);
                    Log.info("" + targetName, "target");
                    Log.info("" + sourceName, "source");

                    EMADynamicConnectorSymbol sym = new EMADynamicConnectorSymbol(targetName);
                    sym.setSource(sourceName);
                    sym.setTarget(targetName);


                    setDynamicInConnectorSourceTarget(sym, node.getSource().getQualifiedNameWithArray(), target.getQualifiedNameWithArray());

                    Log.info(sym.getTarget(), "TARGETNAME SET TO");

                    symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
                    ++counter;
                }
            }
            // TODO enable checking again if it is fixed
            /* if(counter!=targetnum) { Log.error("source port number "+ counter +" and target port num"+
             * targetnum+" don't match"); } */
        }
        if (node.getTargets().isHASH()) {
            String sourceName = getPortName(node.getSource(), symbolTableCreator).get(0);
            EMADynamicConnectorSymbol sym = new EMADynamicConnectorSymbol("#");
            sym.setSource(sourceName);
            sym.setTarget("#");
            symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
        }
    }



    //</editor-fold>

    public void setDynamicInConnectorSourceTarget(EMADynamicConnectorSymbol connector, ASTQualifiedNameWithArray source, ASTQualifiedNameWithArray target){
        ASTArrayAccess compAccess = (ASTArrayAccess) target.getCompArrayOpt().orElse(null);
        ASTArrayAccess portAccess = (ASTArrayAccess) target.getPortArrayOpt().orElse(null);

        if(compAccess != null){
            connector.setDynamicTargetNewComponent(compAccess.isDynamicNewPort());
        }
        if(portAccess != null){
            connector.setDynamicTargetNewPort(portAccess.isDynamicNewPort());
        }

        if(source != null){
            compAccess = (ASTArrayAccess) source.getCompArrayOpt().orElse(null);
            portAccess = (ASTArrayAccess) source.getPortArrayOpt().orElse(null);

            if(compAccess != null){
                connector.setDynamicSourceNewComponent(compAccess.isDynamicNewPort());
            }
            if(portAccess != null){
                connector.setDynamicSourceNewPort(portAccess.isDynamicNewPort());
            }
        }


    }

    //</editor-fold>


    //<editor-fold desc="Resolution">


    public static int handleSizeResolution(ASTSubComponent node, int index) {
        int counter = 0;
        if (node.getType() instanceof ASTSimpleReferenceType) {
            if (((ASTSimpleReferenceType) node.getType()).getTypeArgumentsOpt().isPresent()) {
                for (ASTTypeArgument typeArgument : ((ASTSimpleReferenceType) node.getType())
                        .getTypeArgumentsOpt().get().getTypeArgumentList()) {
                    if (typeArgument instanceof ASTUnitNumberTypeArgument) {
                        Log.debug("" + ((ASTUnitNumberTypeArgument) typeArgument).getNumberWithUnit().getNumber()
                                .get().intValue(), "New Resolution Value:");
                        if (counter == index)
                            return ((ASTUnitNumberTypeArgument) typeArgument).getNumberWithUnit().getNumber().get()
                                    .intValue();
                        ++counter;
                    }
                }
            }
        }
        return -1;
    }

    //</editor-fold>
}
