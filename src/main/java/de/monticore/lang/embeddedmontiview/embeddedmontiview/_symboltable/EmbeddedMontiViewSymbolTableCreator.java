/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.ast.ASTNode;
//import de.monticore.common.common._ast.ASTStereoValue;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.*;
import de.monticore.lang.embeddedmontiview.embeddedmontiview.types.TypesHelper;
import de.monticore.lang.embeddedmontiview.embeddedmontiview.types.TypesPrinter;
import de.monticore.lang.embeddedmontiview.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiview.helper.Timing;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.common2._ast.ASTArrayAccess;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.common2._ast.ASTStereoValue;
import de.monticore.lang.monticar.mcexpressions._ast.ASTExpression;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._ast.ASTRanges;
import de.monticore.lang.monticar.ranges._ast.ASTUnitNumberExpression;
import de.monticore.lang.monticar.resolution._ast.ASTResolutionDeclaration;
import de.monticore.lang.monticar.resolution._ast.ASTTypeArgument;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbolReference;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbolReference;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarSymbolFactory;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTComplexArrayType;
import de.monticore.lang.monticar.types2._ast.ASTComplexReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTImportStatement;
import de.monticore.lang.monticar.types2._ast.ASTReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTSimpleReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.lang.monticar.types2._ast.ASTTypeNameResolutionDeclaration;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration;
import de.monticore.lang.monticar.types2._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.types2._ast.ASTUnitNumberTypeArgument;
import de.monticore.lang.monticar.types2._ast.ASTWildcardType;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.modifiers.BasicAccessModifier;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.symboltable.types.references.TypeReference;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.StringTransformations;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.*;

import siunit.monticoresiunit.si._ast.ASTUnitNumber;

import static de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiArcExpandedComponentInstanceSymbolCreator.getGlobalScope;

/**
 * Visitor that creats the symboltable of an EmbeddedMontiArc AST.
 *
 */
public class EmbeddedMontiViewSymbolTableCreator
    extends EmbeddedMontiViewSymbolTableCreatorTOP {

  private String compilationUnitPackage = "";
  private EmbeddedMontiArcExpandedComponentInstanceSymbolCreator instanceSymbolCreator = new EmbeddedMontiArcExpandedComponentInstanceSymbolCreator();
  // extra stack of components that is used to determine which components are inner components.
  private Stack<ViewComponentSymbol> componentStack = new Stack<>();
  private ViewSymbol currentView = null;
  private List<ImportStatement> currentImports = new ArrayList<>();
  private MontiCarSymbolFactory jSymbolFactory = new MontiCarSymbolFactory();

  protected boolean aboartVisitComponent = false;

  protected boolean autoInstantiate = false;

  public EmbeddedMontiViewSymbolTableCreator(final ResolvingConfiguration resolverConfig, final MutableScope enclosingScope) {
    super(resolverConfig, enclosingScope);
  }

  public EmbeddedMontiViewSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
    super(resolvingConfig, scopeStack);
  }

  @Override
  public void visit(ASTEMVCompilationUnit compilationUnit) {
    Log.debug("Building Symboltable for View: " + compilationUnit.getView().getName(), EmbeddedMontiViewSymbolTableCreator.class.getSimpleName());
    compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackage());

    // imports
    List<ImportStatement> imports = new ArrayList<>();
    for (ASTImportStatement astImportStatement : compilationUnit.getImportStatements()) {
      String qualifiedImport = Names.getQualifiedName(astImportStatement.getImportList());
      ImportStatement importStatement = new ImportStatement(qualifiedImport, astImportStatement.isStar());
      imports.add(importStatement);
    }
    EMAJavaHelper.addJavaDefaultImports(imports);

    ArtifactScope artifactScope = new EmbeddedMontiArcArtifactScope(Optional.empty(), compilationUnitPackage, imports);
    this.currentImports = imports;
    putOnStack(artifactScope);
  }

  public void endVisit(ASTEMVCompilationUnit node) {
    // TODO clean up component types from references to inner components
    // cleanUpReferences();

    // artifact scope
    removeCurrentScope();

    if (aboartVisitComponent) {
      return;
    }
    // creates all instances which are created through the top level component
    System.out.println("endVisit of " + node.getView().getSymbol().get().getFullName()); //,"MontiArcSymbolTableCreator");
    //    new Error().printStackTrace();
    //instanceSymbolCreator.createInstances((ViewSymbol) (Log.errorIfNull(node.getView().getSymbol().orElse(null))));
  }

  /**
   * handles typeref creation of an SIUnitRangeType from ASTRange
   */
  private MCTypeReference<? extends MCTypeSymbol> initTypeRefASTRange(ASTPort node, StringBuilder typeName, ASTRange astType) {
    typeName.append("SIUnitRangesType");
    Log.debug(astType.toString(), "Type:");
    Log.debug(typeName.toString(), "TypeName:");

    SIUnitRangesSymbolReference ref = SIUnitRangesSymbolReference.constructSIUnitRangesSymbolReference(astType);
    return ref;
  }

  /**
   * handles typeref creation of an SIUnitRangesType from ASTRanges
   */
  private MCTypeReference<? extends MCTypeSymbol> initTypeRefASTRanges(ASTPort node, StringBuilder typeName, ASTRanges astType) {
    typeName.append("SIUnitRangesType");
    Log.debug(astType.toString(), "Type:");
    Log.debug(typeName.toString(), "TypeName:");

    SIUnitRangesSymbolReference ref = SIUnitRangesSymbolReference.constructSIUnitRangesSymbolReference(astType.getRanges());
    return ref;
  }

  /**
   * handles typeref creation of a GeneralType
   */
  private MCTypeReference<? extends MCTypeSymbol> initTypeRefGeneralType(ASTPort node, StringBuilder typeName, ASTType astType) {
    MCTypeReference<? extends MCTypeSymbol> typeRef = null;
    typeName.append(ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(astType));
    //Log.debug(astType.toString(),"TYPE:");
    //Log.debug(typeName,"TYPEName:");
    typeRef = new CommonMCTypeReference<MCTypeSymbol>(typeName.toString(), MCTypeSymbol.KIND, currentScope().get());
    typeRef.setDimension(TypesHelper.getArrayDimensionIfArrayOrZero(astType));
    addTypeArgumentsToTypeSymbol(typeRef, astType);
    return typeRef;
  }

  /**
   * returns an initialized type reference for a port
   *
   * @param node     the node which is
   * @param typeName
   * @param astType
   * @return
   */
  private MCTypeReference<? extends MCTypeSymbol> initTypeRef(ASTPort node, StringBuilder typeName, ASTType astType) {
    if (node.getType().get() instanceof ASTRange) {
      return initTypeRefASTRange(node, typeName, (ASTRange) astType);
    }
    else if (node.getType().get() instanceof ASTRanges) {
      return initTypeRefASTRanges(node, typeName, (ASTRanges) astType);
    }
    Log.debug(node.getName().isPresent() ? node.getName().get() : "unnamed port" + " " + astType.toString(), "info");
    return initTypeRefGeneralType(node, typeName, astType);
  }

  /**
   * creates the PortSymbols that belong to a ViewPortArraySymbol
   */
  private void portCreationIntLiteralPresent(ASTPort node, ViewPortArraySymbol pas, String name, Optional<MCTypeReference<? extends MCTypeSymbol>> typeRef) {
    //int num = node.getIntLiteral().get().getValue();
    Log.debug(node.toString(), "ASTPort");
    int num = 0;
    if (node.getUnitNumberResolution().isPresent() && node.getUnitNumberResolution().get().getUnitNumber().isPresent()) {
      num = node.getUnitNumberResolution().get().getNumber().get().intValue();
    }
    else {
      Log.debug("No UnitNumberResolution/UnitNumber present!", "ASTPort");
    }
    pas.setDimension(num);
    for (int i = 1; i <= num; ++i) {
      String nameWithArray = name + "[" + Integer.toString(i) + "]";
      ViewPortSymbol sym = new ViewPortSymbol(nameWithArray);

      Log.debug(nameWithArray, "nameWithArray");

      sym.setTypeReference(typeRef);
      sym.setDirection(node.isIncoming());

      if (node.getStereotype().isPresent()) {
        for (ASTStereoValue st : node.getStereotype().get().getValues()) {
          sym.addStereotype(st.getName(), st.getValue());
        }
      }
      addToScopeAndLinkWithNode(sym, node);
    }
  }

  private void portCreation(ASTPort node, ViewPortArraySymbol pas, String name, Optional<MCTypeReference<? extends MCTypeSymbol>> typeRef) {
    if (node.getUnitNumberResolution().isPresent()) {
      portCreationIntLiteralPresent(node, pas, name, typeRef);
    }
    else {
      // create ViewPortSymbol with same content as ViewPortArraySymbol
      createPort(node, name, node.isIncoming(), pas.getStereotype(), typeRef);
    }
  }

  public void createPort(String name, boolean isIncoming, Map<String, Optional<String>> stereoType, Optional<MCTypeReference<? extends MCTypeSymbol>> typeRef) {
    ViewPortSymbol ps = new ViewPortSymbol(name);

    ps.setTypeReference(typeRef);
    ps.setDirection(isIncoming);

    stereoType.forEach(ps::addStereotype);

    addToScope(ps);
  }

  public void createPort(ASTPort node, String name, boolean isIncoming, Map<String, Optional<String>> stereoType, Optional<MCTypeReference<? extends MCTypeSymbol>> typeRef) {
    ViewPortSymbol ps = new ViewPortSymbol(name);

    ps.setTypeReference(typeRef);
    ps.setDirection(isIncoming);

    stereoType.forEach(ps::addStereotype);

    addToScopeAndLinkWithNode(ps, node);
  }

  public String doPortResolution(ASTPort node) {
    String name = null;
    if (node.getUnitNumberResolution().isPresent()) {
      ASTUnitNumberResolution unitNumberResolution = node.getUnitNumberResolution().get();
      name = unitNumberResolution.doResolution(componentStack.peek().getResolutionDeclarationSymbols());

    }
    return name;
  }

  @Override
  public void visit(ASTPort node) {

    String nameTO = doPortResolution(node);
    String name = "";
    Optional<MCTypeReference<? extends MCTypeSymbol>> typeRef;
    StringBuilder typeName = new StringBuilder();

    if (node.getType().isPresent()) {
      ASTType astType = node.getType().get();
      typeRef = Optional.of(initTypeRef(node, typeName, astType));
    }
    else {
      assert node.isAnonymousType();
      typeRef = Optional.empty();
    }

    if (node.getName().isPresent()) {
      name = node.getName().get();
    }
    else if (node.isAnonymousName()) {
      name = "?";
    }
    else {
      assert typeRef.isPresent();
      name = node.getName().orElse(StringTransformations.uncapitalize(typeName.toString()));
    }

    ViewPortArraySymbol pas = new ViewPortArraySymbol(name, nameTO);
    pas.setTypeReference(typeRef);
    pas.setDirection(node.isIncoming());

    // stereotype
    if (node.getStereotype().isPresent()) {
      for (ASTStereoValue st : node.getStereotype().get().getValues()) {
        pas.addStereotype(st.getName(), st.getValue());
      }
    }

    addToScopeAndLinkWithNode(pas, node);

    portCreation(node, pas, name, typeRef);
  }

  private List<String> getPortName(ASTQualifiedNameWithArray portName) {
    List<String> names = new ArrayList<String>();

    List<String> compNameParts = getComponentNameParts(portName);

    List<String> portNameParts;
    portNameParts = getPortNameParts(portName);

    Log.debug("portName: " + portName + " " + compNameParts.size(), "CompNameParts");
    Log.debug("" + portNameParts.size(), "PortNameParts");
    for (String compNamePart : compNameParts) {
      for (String portNamePart : portNameParts) {
        String curName = compNamePart + portNamePart;

        names.add(curName);
      }
    }

    return names;
  }

  private List<String> getComponentNameParts(ASTQualifiedNameWithArray portName) {
    List<String> names = new ArrayList<String>();
    String name = "";
    if (portName.getCompName().isPresent()) {
      name += portName.getCompName().get();
      if (portName.getCompArray().isPresent()) {
        if (portName.getCompArray().get().getIntLiteral().isPresent()) {
          name += "[" + portName.getCompArray().get().getIntLiteral().get().getNumber().toString() + "]";
          name += ".";
          names.add(name);
        }
        else if (portName.getCompArray().get().getLowerbound().isPresent()) {
          names = getmnCompNameParts(name, portName);
        }
        else {
          int size = countComponentArrayInstances(name);
          for (int i = 1; i <= size; ++i) {
            String instanceName = name;
            instanceName += "[" + i + "].";
            names.add(instanceName);
          }
        }
      }
      else {
        names.add(portName.getCompName().get() + ".");
      }
    }
    else {
      names.add("");
    }
    return names;
  }

  private List<String> getmnCompNameParts(String name, ASTQualifiedNameWithArray portName) {
    List<String> names = new ArrayList<String>();
    int lower = portName.getCompArray().get().getLowerbound().get().getNumber().get().intValue();
    int upper = portName.getCompArray().get().getUpperbound().get().getNumber().get().intValue();
    for (int i = lower; i <= upper; ++i) {
      String instanceName = name;
      instanceName += "[" + i + "].";
      names.add(instanceName);
    }
    return names;
  }

  private List<String> getPortNameParts(ASTQualifiedNameWithArray portName) {
    return getPortNameParts(portName, 0);
  }

  private List<String> getPortNameParts(ASTQualifiedNameWithArray portName, int amountSources) {

    List<String> names = new ArrayList<String>();
    String name = "";
    //ignore for now
    if (portName.getCompName().isPresent())
      name += portName.getCompName().get() + ".";
    name = portName.getPortName();
    if (portName.getPortArray().isPresent()) {
      if (portName.getPortArray().get().getIntLiteral().isPresent()) {
        name += "[" + portName.getPortArray().get().getIntLiteral().get().getNumber() + "]";
        names.add(name);
      }
      else if (portName.getPortArray().get().getLowerbound().isPresent()) {
        names = getmnPortNameParts(name, portName);
      }
      else {

        int size = countPortArrayInstances(name, portName.getCompName().orElse(null));

        Log.debug("Size" + size, "PortNameParts");
        for (int i = 1; i <= size; ++i) {
          String instanceName = name;

          instanceName += "[" + i + "]";

          names.add(instanceName);
        }
      }
    }
    else {
      Log.debug("No PortArrayName was specified", "PortArray");
      names.add(portName.getPortName());
    }
    return names;
  }

  private List<String> getmnPortNameParts(String name, ASTQualifiedNameWithArray portName) {
    List<String> names = new ArrayList<String>();
    int lower = portName.getPortArray().get().getLowerbound().get().getNumber().get().intValue();
    int upper = portName.getPortArray().get().getUpperbound().get().getNumber().get().intValue();
    for (int i = lower; i <= upper; ++i) {
      String instanceName = name;
      instanceName += "[" + i + "]";
      names.add(instanceName);
      Log.debug("Name:", "Added MNPortName");
    }
    return names;
  }

  private int countPortArrayInstances(String portName, String compName) {
    MutableScope curScope = currentScope().get();
    boolean present = true;
    int counter = 0;

    while (present) {
      present = curScope.resolve(portName + "[" + (counter + 1) + "]", ViewPortSymbol.KIND).isPresent();
      if (present)
        ++counter;
      else {
        Log.debug("Could not resolve " + portName + "[" + (counter + 1) + "]", "countPortArrayInstances");
      }
    }
    if (counter == 0) {
      //TODO
      present = true;
      Log.debug("compInstanceName: " + compName, "Resolving");
      ViewComponentInstanceSymbol symbol = curScope.<ViewComponentInstanceSymbol>resolve(compName, ViewComponentInstanceSymbol.KIND).get();
      for (ViewPortSymbol viewPortSymbol : symbol.getComponentType().getAllPorts()) {

        Log.debug(viewPortSymbol.toString(), "PortInfo");
        if (viewPortSymbol.getNameWithoutArrayBracketPart().startsWith(portName)) {
          ++counter;
        }
      }

    }

    return counter;
  }

  private int countComponentArrayInstances(String componentName) {
    MutableScope curScope = currentScope().get();
    boolean present = true;
    int counter = 0;
    Log.debug("" + componentName, "RESOLVING");
    while (present) {
      present = curScope.resolve(componentName + "[" + (counter + 1) + "]", ViewComponentInstanceSymbol.KIND).isPresent();
      if (present)
        ++counter;
    }
    return counter;
  }

  private void nonConstantPortSetup_Connector(List<String> sourceNames, ASTConnector node) {
    Log.debug("" + sourceNames.size(), "SourcePorts");
    int counter = 0, targetnum = 0;
    for (ASTQualifiedNameWithArray target : node.getTargets()) {
      counter = 0;
      targetnum = 0;
      for (String sourceName : sourceNames) {
        List<String> targetNames = getPortName(target);
        //        targetnum = targetNames.size();
        String targetName = targetNames.get(counter);
        Log.debug("" + targetName, "target");
        Log.debug("" + sourceName, "source");

        ViewConnectorSymbol sym = new ViewConnectorSymbol(sourceName + targetName);
        sym.setSource(sourceName);
        sym.setTarget(targetName);
        Log.debug(sym.getTarget(), "TARGETNAME SET TO");
        // stereotype
        if (node.getStereotype().isPresent()) {
          for (ASTStereoValue st : node.getStereotype().get().getValues()) {
            sym.addStereotype(st.getName(), st.getValue());
          }
        }
        addToScopeAndLinkWithNode(sym, node);
        ++counter;
      }
      //TODO enable checking again if it is fixed
            /*if(counter!=targetnum)
            {
                Log.error("source port number "+ counter +" and target port num"+ targetnum+" don't match");
            }*/
    }
  }

  private void nonConstantPortSetup_Effector(List<String> sourceNames, ASTEffector node) {
    Log.debug("" + sourceNames.size(), "SourcePorts");
    int counter = 0, targetnum = 0;
    for (ASTQualifiedNameWithArray target : node.getTargets()) {
      counter = 0;
      targetnum = 0;
      for (String sourceName : sourceNames) {
        List<String> targetNames = getPortName(target);
        //        targetnum = targetNames.size();
        String targetName = targetNames.get(counter);
        Log.debug("" + targetName, "target");
        Log.debug("" + sourceName, "source");

        ViewEffectorSymbol sym = new ViewEffectorSymbol(sourceName  + targetName);
        sym.setSource(sourceName);
        sym.setTarget(targetName);
        Log.debug(sym.getTarget(), "TARGETNAME SET TO");
        // stereotype
        if (node.getStereotype().isPresent()) {
          for (ASTStereoValue st : node.getStereotype().get().getValues()) {
            sym.addStereotype(st.getName(), st.getValue());
          }
        }
        addToScopeAndLinkWithNode(sym, node);
        ++counter;
      }
      //TODO enable checking again if it is fixed
      /*if(counter!=targetnum)
            {
                Log.error("source port number "+ counter +" and target port num"+ targetnum+" don't match");
            }*/
    }
  }

  @Override
  public void visit(ASTConnector node) {
    ASTQualifiedNameWithArray portName = node.getSource();
    List<String> sourceNames = getPortName(portName);
    // Log.debug(node.getSource().get().toString(),"port content");
    nonConstantPortSetup_Connector(sourceNames, node);
  }

  @Override
  public void visit(ASTEffector node) {
    ASTQualifiedNameWithArray portName = node.getSource();
    List<String> sourceNames = getPortName(portName);
    // Log.debug(node.getSource().get().toString(),"port content");
    nonConstantPortSetup_Effector(sourceNames, node);
  }

  @Override
  public void visit(ASTMontiArcAutoInstantiate node) {
    autoInstantiate = node.isOn();
  }

  public int handleSizeResolution(ASTSubComponent node) {

    if (node.getType() instanceof ASTSimpleReferenceType) {
      if (((ASTSimpleReferenceType) node.getType()).getTypeArguments().isPresent()) {
        for (ASTTypeArgument typeArgument : ((ASTSimpleReferenceType) node.getType()).getTypeArguments().get().getTypeArguments()) {
          if (typeArgument instanceof ASTUnitNumberTypeArgument) {
            Log.debug("" + ((ASTUnitNumberTypeArgument) typeArgument).getUnitNumber().getNumber().get().intValue(), "New Resolution Value:");

            return ((ASTUnitNumberTypeArgument) typeArgument).getUnitNumber().getNumber().get().intValue();
          }
        }
      }
    }

    return -1;
  }

  public void doSubComponentInstanceResolution(ASTSubComponentInstance node, ViewComponentSymbolReference componentSymbolReference) {
    if (node.getUnitNumberResolution().isPresent()) {
      ASTUnitNumberResolution unitNumberResolution = node.getUnitNumberResolution().get();
      ASTUnitNumber toSet = null;
      if (unitNumberResolution.getUnitNumber().isPresent()) {
        toSet = unitNumberResolution.getUnitNumber().get();

      }
      else if (unitNumberResolution.getName().isPresent()) {

        ResolutionDeclarationSymbol resDeclSym = componentStack.peek().getResolutionDeclarationSymbol(unitNumberResolution.getName().get()).get();
        Log.debug(resDeclSym.getASTResolution().toString(), "Found ResolutionDeclarationSymbol:");
        toSet = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getUnitNumber().get();

        Log.debug("" + toSet.getNumber().get().intValue(), "ToSet Number:");
      }
      node.getUnitNumberResolution().get().setUnit(toSet.getUnit().get());
      node.getUnitNumberResolution().get().setNumber(toSet.getNumber().get());

      Log.debug("" + node.getUnitNumberResolution().get().getNumber().get().intValue(), "SubComponentResolution Number:");
    }
  }

  public void setActualResolutionDeclaration(ASTSubComponent node, ViewComponentSymbolReference componentSymbolReference) {
    int size = handleSizeResolution(node);
    if (size > 0 && componentSymbolReference.getResolutionDeclarationSymbols().size() > 0) {
      if (componentSymbolReference.getResolutionDeclarationSymbols().get(0).getASTResolution() instanceof ASTUnitNumberResolution) {
        Log.debug(size + "", "Set new Resolution");
        ((ASTUnitNumberResolution) componentSymbolReference.getResolutionDeclarationSymbols().get(0).getASTResolution()).setNumber(Rational.valueOf("" + size));
      }
    }
    else {
      for (int i = 0; i < componentSymbolReference.getResolutionDeclarationSymbols().size(); ++i) {
        Rational numberToSetTo = ((ASTUnitNumberResolution) componentSymbolReference.getReferencedSymbol().getResolutionDeclarationSymbols().get(i).getASTResolution()).getNumber().get();
        ((ASTUnitNumberResolution) componentSymbolReference.getResolutionDeclarationSymbols().get(i).getASTResolution()).setNumber(numberToSetTo);
      }
    }
  }

  @Override
  public void visit(ASTSubComponent node) {
    String referencedCompName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(node.getType());

    // String refCompPackage = Names.getQualifier(referencedCompName);
    String simpleCompName = Names.getSimpleName(referencedCompName);

    ViewComponentSymbolReference componentTypeReference = new ViewComponentSymbolReference(referencedCompName, currentScope().get(), this);

    //set actual Resolution values
    setActualResolutionDeclaration(node, componentTypeReference);

    // actual type arguments
    //TODO enable if needed
    addTypeArgumentsToTypeSymbol(componentTypeReference, node.getType());

    // ref.setPackageName(refCompPackage);

    // TODO internal representation of ValueSymbol ? that was heavily based on CommonValues
    // language and its expressions, but we use JavaDSL.
    List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs = new ArrayList<>();
        /*for (ASTExpression arg : node.getArguments()) {
            String value = new JavaDSLPrettyPrinter(new IndentPrinter()).prettyprint(arg);
            value = value.replace("\"", "\\\"").replace("\n", "");
            configArgs.add(new ValueSymbol<>(value, Kind.Expression));
        }*/

    componentTypeReference.fixResolutions(this);
    // instances

    if (!node.getInstances().isEmpty()) {
      // create instances of the referenced components.
      for (ASTSubComponentInstance i : node.getInstances()) {
        //For generic type resolution Example: <N1 n=4> with instance being <6> to change value of n accordingly
        doSubComponentInstanceResolution(i, componentTypeReference);
        Log.debug(node.getType().toString(), "Pre Handle Size:");

        if (i.getUnitNumberResolution().isPresent()) {
          int size = i.getUnitNumberResolution().get().getNumber().get().intValue();
          Log.debug(node.getType().toString(), "First: ");
          Log.debug(node.getType().toString(), "Second: ");

          for (int ii = 1; ii <= size; ++ii) {
            createInstance(i.getName() + "[" + ii + "]", node, componentTypeReference, configArgs);
          }
        }
        else {
          createInstance(i.getName(), node, componentTypeReference, configArgs);
        }
      }
    }
    else {
      // auto instance because instance name is missing
      createInstance(StringTransformations.uncapitalize(simpleCompName), node, componentTypeReference, new ArrayList<>());
    }

    node.setEnclosingScope(currentScope().get());
  }

  /**
   * Creates the instance and adds it to the symTab.
   */

  private void createInstance(String name, ASTNode node, ViewComponentSymbolReference componentTypeReference, List<ValueSymbol<TypeReference<TypeSymbol>>> configArguments) {
    ViewComponentInstanceSymbol instance = new ViewComponentInstanceSymbol(name, componentTypeReference);
    configArguments.forEach(v -> instance.addConfigArgument(v));
    // create a subscope for the instance
    addToScopeAndLinkWithNode(instance, node);
    Log.debug(currentScope().get().toString(), "SubComponentInstance Scope");
    // remove the created instance's scope
    removeCurrentScope();
    Log.debug(name, "created SubComponentInstance:");
  }

  @Override
  public void handle(ASTComponent node) {
    getRealThis().visit(node);
    if (!aboartVisitComponent) {
      getRealThis().traverse(node);
      getRealThis().endVisit(node);
    }
  }

  private void handleResolutionDeclaration(ViewComponentSymbol typeSymbol, Optional<ASTTypeParameters> optionalTypeParameters, Scope currentScope, ASTComponent node) {
    if (optionalTypeParameters.isPresent()) {
      ASTTypeParameters astTypeParameters = optionalTypeParameters.get();
      for (ASTTypeVariableDeclaration astTypeParameter : astTypeParameters.getTypeVariableDeclarations()) {
        if (astTypeParameter.resolutionDeclarationIsPresent() && astTypeParameter.getResolutionDeclaration().get() instanceof ASTTypeNameResolutionDeclaration) {
          Log.debug(astTypeParameter.toString(), "Resolution Declaration:");
          ASTResolutionDeclaration astResDecl = astTypeParameter.getResolutionDeclaration().get();

          ResolutionDeclarationSymbolReference resDeclSymRef;
          resDeclSymRef = ResolutionDeclarationSymbolReference.constructResolutionDeclSymbolRef(((ASTTypeNameResolutionDeclaration) astResDecl).getName(), ((ASTTypeNameResolutionDeclaration) astResDecl).getResolution());

          Log.debug(resDeclSymRef.getNameToResolve(), "Added ResolutionDeclarationSymbol with name: ");
          typeSymbol.addResolutionDeclarationSymbol(resDeclSymRef);
          //TODO Resolution maybe link with node
          addToScopeAndLinkWithNode(resDeclSymRef, astTypeParameter);
          currentScope().get().add(resDeclSymRef);
        }
      }
    }
  }

  @Override
  public void visit(ASTView node) {
    assert componentStack.isEmpty() : "Views may not be nested!";

    String viewName = node.getName();
    String viewPackageName = compilationUnitPackage;

    ViewSymbol view = new ViewSymbol(viewName);
    view.setImports(currentImports);
    view.setPackageName(viewPackageName);

    currentView = view;
    addToScopeAndLinkWithNode(view, node);
  }

  @Override
  public void visit(ASTComponent node) {
    String componentName = node.getName();

    String componentPackageName = compilationUnitPackage;

    ViewComponentSymbol component = new ViewComponentSymbol(componentName);
    component.setImports(currentImports);

    if (!componentStack.isEmpty()) {
      // inner component uses its parents component full name as package
      componentPackageName = componentStack.peek().getFullName();
    }

    component.setIsInnerComponent(true); //always inside iof a view, at least
    component.setPackageName(componentPackageName);

    //Handle ResolutionDeclaration of stuff like <N1 n=5>
    handleResolutionDeclaration(component, node.getHead().getGenericTypeParameters(), currentScope().get(), node);

    Log.debug(component.toString(), "ComponentPreGeneric");
    // generic type parameters
    EMAJavaHelper.addTypeParametersToType(component, node.getHead().getGenericTypeParameters(), currentScope().get());

    Log.debug(component.toString(), "ComponentPostGeneric");
    // parameters
    //Log.debug(node.getHead().toString(),"ASTComponentHead");
    setParametersOfComponent(component, node.getHead());
    //Log.debug(component.toString(),"ComponentPostParam");

    // super component
    if (node.getHead().getSuperComponent().isPresent()) {
      ASTReferenceType superCompRef = node.getHead().getSuperComponent().get();
      String superCompName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(superCompRef);

      ViewComponentSymbolReference ref = new ViewComponentSymbolReference(superCompName, currentScope().get());
      ref.setAccessModifier(BasicAccessModifier.PUBLIC);
      // actual type arguments
      addTypeArgumentsToTypeSymbol(ref, superCompRef);

      component.setSuperComponent(Optional.of(ref));
    }

    // stereotype
    //    if (node.getStereotype().isPresent()) {
    //      for (ASTStereoValue st : node.getStereotype().get().getValues()) {
    //        component.addStereotype(st.getName(), st.getValue());
    //      }
    //    }
    //instead of stereotype, mark as atomic or not.
    component.setMarkedAtomic(node.isAtomicTagIsPresent());

    // timing
    component.setBehaviorKind(Timing.getBehaviorKind(node));

    componentStack.push(component);

    addToScopeAndLinkWithNode(component, node);

    // TODO this is a hack to avoid loading one component symbol twice
    // --> must be changed in future
    Collection<Symbol> c = getGlobalScope(currentScope().get()).resolveDownMany(component.getFullName(), ViewComponentSymbol.KIND);
    if (c.size() > 1) {
      aboartVisitComponent = true;
      component.getEnclosingScope().getAsMutableScope().removeSubScope(component.getSpannedScope().getAsMutableScope());

      return;
    }

  }

  public void visit(ASTInterface node) {
    if (node.isInterfaceCompleteTagIsPresent()) {
      componentStack.peek().setIsInterfaceComplete(true);
    }
  }

  private void setParametersOfComponent(final ViewComponentSymbol viewComponentSymbol, final ASTComponentHead astMethod) {
    Log.debug(viewComponentSymbol.toString(), "ComponentPreParam");
    Log.debug(astMethod.toString(), "ASTComponentHead");
    for (ASTParameter astParameter : astMethod.getParameters()) {
      final String paramName = astParameter.getName();
      Log.debug(astParameter.toString(), "ASTParam");
      int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astParameter.getType());

      //TODO enable if needed and remove line below
      MCTypeReference<? extends MCTypeSymbol> paramTypeSymbol = new MontiCarTypeSymbolReference(TypesPrinter.printTypeWithoutTypeArgumentsAndDimension(astParameter.getType()), currentScope().get(), dimension);

      addTypeArgumentsToTypeSymbol(paramTypeSymbol, astParameter.getType());

      final MCFieldSymbol parameterSymbol = jSymbolFactory.createFormalParameterSymbol(paramName, (MontiCarTypeSymbolReference) paramTypeSymbol);
      viewComponentSymbol.addConfigParameter(parameterSymbol);
    }
    Log.debug(viewComponentSymbol.toString(), "ComponentPostParam");
  }

  private boolean needsInstanceCreation(ASTComponent node, ViewComponentSymbol symbol) {
    boolean instanceNameGiven = node.getInstanceName().isPresent();
    boolean autoCreationPossible = symbol.getFormalTypeParameters().size() == 0;

    return autoInstantiate && (instanceNameGiven || autoCreationPossible);
  }

  @Override
  public void endVisit(ASTComponent node) {
    ViewComponentSymbol component = componentStack.pop();

    removeCurrentScope();

    // for inner components the symbol must be fully created to reference it. Hence, in endVisit we
    // can reference it and put the instance of the inner component into its parent scope.

    if (component.isInnerComponent()) {
      String referencedComponentTypeName = component.getFullName();
      ViewComponentSymbolReference refEntry = new ViewComponentSymbolReference(referencedComponentTypeName, component.getSpannedScope());
      refEntry.setReferencedComponent(Optional.of(component));

      if (needsInstanceCreation(node, component)) {
        // create instance
        String instanceName = node.getInstanceName().orElse(StringTransformations.uncapitalize(component.getName()));

        if (node.getActualTypeArgument().isPresent()) {
          setActualTypeArguments(refEntry, node.getActualTypeArgument().get().getTypeArguments());
        }

        ViewComponentInstanceSymbol instanceSymbol = new ViewComponentInstanceSymbol(instanceName, refEntry);
        Log.debug("Created component instance " + instanceSymbol.getName() + " referencing component type " + referencedComponentTypeName, EmbeddedMontiViewSymbolTableCreator.class.getSimpleName());

        addToScope(instanceSymbol);
      }

      // collect inner components that do not have generic types or a
      // configuration
      if (component.getFormalTypeParameters().isEmpty() && component.getConfigParameters().isEmpty() && !node.getInstanceName().isPresent()) {
        // Pair<ViewComponentSymbol, ASTComponent> p = new Pair<>(owningComponent, node);
        // TODO store as inner component?
        // innerComponents.put(component, p);
      }
    }
  }

  @Override
  public void endVisit(ASTView node) {
    removeCurrentScope();
  }

  // TODO remove after GV's refactoring of such methodology to mc4/types.
  @Deprecated
  private void addTypeArgumentsToTypeSymbol(MCTypeReference<? extends MCTypeSymbol> typeReference, ASTType astType) {
    if (astType instanceof ASTSimpleReferenceType) {
      ASTSimpleReferenceType astSimpleReferenceType = (ASTSimpleReferenceType) astType;
      if (!astSimpleReferenceType.getTypeArguments().isPresent()) {
        return;
      }
      List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
      for (ASTTypeArgument astTypeArgument : astSimpleReferenceType.getTypeArguments().get().getTypeArguments()) {
        addActualTypeArguments(astTypeArgument, actualTypeArguments, typeReference);
        typeReference.setActualTypeArguments(actualTypeArguments);
      }
    }
    else if (astType instanceof ASTComplexReferenceType) {
      ASTComplexReferenceType astComplexReferenceType = (ASTComplexReferenceType) astType;
      for (ASTSimpleReferenceType astSimpleReferenceType : astComplexReferenceType.getSimpleReferenceTypes()) {
        // TODO
        /* ASTComplexReferenceType represents types like class or interface types which always have
         * ASTSimpleReferenceType as qualification. For example: a.b.c<Arg>.d.e<Arg> */
      }
    }
    else if (astType instanceof ASTComplexArrayType) {
      ASTComplexArrayType astComplexArrayType = (ASTComplexArrayType) astType;
      // references to types with dimension>0, e.g., String[]
      addTypeArgumentsToTypeSymbol(typeReference, astComplexArrayType.getComponentType());
      int dimension = astComplexArrayType.getDimensions();
      typeReference.setDimension(dimension);
    }
  }

  private void addActualTypeArguments(ASTTypeArgument astTypeArgument, List<ActualTypeArgument> actualTypeArguments, MCTypeReference<? extends MCTypeSymbol> typeReference) {
    addActualTypeArguments(astTypeArgument, actualTypeArguments, typeReference.toString());
  }

  private void addActualTypeArguments(ASTTypeArgument astTypeArgument, List<ActualTypeArgument> actualTypeArguments, ViewComponentSymbolReference typeReference) {
    addActualTypeArguments(astTypeArgument, actualTypeArguments, typeReference.toString());
  }

  private void addActualTypeArguments(ASTTypeArgument astTypeArgument, List<ActualTypeArgument> actualTypeArguments, String typeReferenceString) {
    if (astTypeArgument instanceof ASTWildcardType) {
      addActualTypeArguments_ASTWildcardType((ASTWildcardType) astTypeArgument, actualTypeArguments);
    }
    else if (astTypeArgument instanceof ASTType) {
      addActualTypeArguments_ASTType((ASTType) astTypeArgument, actualTypeArguments);
    }
    else {
      Log.error("0xU0401 Unknown type argument " + astTypeArgument + " of type " + typeReferenceString);
    }
  }

  private void addActualTypeArguments_ASTWildcardType(final ASTWildcardType astWildcardTypeArgument, List<ActualTypeArgument> actualTypeArguments) {
    ASTWildcardType astWildcardType = astWildcardTypeArgument;

    // Three cases can occur here: lower bound, upper bound, no bound
    if (astWildcardType.lowerBoundIsPresent() || astWildcardType.upperBoundIsPresent()) {
      // We have a bound.
      // Examples: Set<? extends Number>, Set<? super Integer>

      // new bound
      boolean lowerBound = astWildcardType.lowerBoundIsPresent();
      ASTType typeBound = lowerBound ? astWildcardType.getLowerBound().get() : astWildcardType.getUpperBound().get();

      int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(typeBound);
      MCTypeReference<? extends MCTypeSymbol> typeBoundSymbolReference = new MontiCarTypeSymbolReference(ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(typeBound), currentScope().get(), dimension);
      // TODO string representation?
      // typeBoundSymbolReference.setStringRepresentation(ArcTypePrinter
      // .printWildcardType(astWildcardType));
      ActualTypeArgument actualTypeArgument = new ActualTypeArgument(lowerBound, !lowerBound, typeBoundSymbolReference);

      // init bound
      addTypeArgumentsToTypeSymbol(typeBoundSymbolReference, typeBound);

      actualTypeArguments.add(actualTypeArgument);
    }
    else {
      // No bound. Example: Set<?>
      actualTypeArguments.add(new ActualTypeArgument(false, false, null));
    }
  }

  private void addActualTypeArguments_ASTType(final ASTType astTypeArgument, List<ActualTypeArgument> actualTypeArguments) {
    // Examples: Set<Integer>, Set<Set<?>>, Set<java.lang.String>
    ASTType astTypeNoBound = (ASTType) astTypeArgument;
    int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astTypeNoBound);
    MCTypeReference<? extends MCTypeSymbol> typeArgumentSymbolReference = new MontiCarTypeSymbolReference(ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(astTypeNoBound), currentScope().get(), dimension);

    // TODO string representation?
    // typeArgumentSymbolReference.setStringRepresentation(TypesPrinter
    // .printType(astTypeNoBound));

    addTypeArgumentsToTypeSymbol(typeArgumentSymbolReference, astTypeNoBound);

    actualTypeArguments.add(new ActualTypeArgument(typeArgumentSymbolReference));
  }

  private void setActualTypeArguments(ViewComponentSymbolReference typeReference, List<ASTTypeArgument> astTypeArguments) {
    List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
    for (ASTTypeArgument astTypeArgument : astTypeArguments) {
      addActualTypeArguments(astTypeArgument, actualTypeArguments, typeReference);
    }
    typeReference.setActualTypeArguments(actualTypeArguments);
  }

  // TODO references to component symbols should not differ from JavaTypeSymbolReference?
  @Deprecated
  private void addTypeArgumentsToTypeSymbol(ViewComponentSymbolReference typeReference, ASTType astType) {
    if (astType instanceof ASTSimpleReferenceType) {
      ASTSimpleReferenceType astSimpleReferenceType = (ASTSimpleReferenceType) astType;
      if (!astSimpleReferenceType.getTypeArguments().isPresent()) {
        return;
      }
      setActualTypeArguments(typeReference, astSimpleReferenceType.getTypeArguments().get().getTypeArguments());
    }
    else if (astType instanceof ASTComplexReferenceType) {
      ASTComplexReferenceType astComplexReferenceType = (ASTComplexReferenceType) astType;
      for (ASTSimpleReferenceType astSimpleReferenceType : astComplexReferenceType.getSimpleReferenceTypes()) {
        // TODO
        /* ASTComplexReferenceType represents types like class or interface types which always have
         * ASTSimpleReferenceType as qualification. For example: a.b.c<Arg>.d.e<Arg> */
      }
    }

  }

  public void removeFromScope(Symbol symbol) {
    currentScope().get().remove(symbol);
  }

  public MutableScope getCurrentScopeAsMutableScope() {
    return currentScope().get();
  }
}
