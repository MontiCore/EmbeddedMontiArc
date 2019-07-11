package de.monticore.lang.montisim.simlang;

import de.monticore.ModelingLanguageFamily;
import de.monticore.ast.ASTNode;
import de.monticore.io.paths.ModelCoordinates;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.montisim.carlang.CoCoCheckerFactory;
import de.monticore.lang.montisim.carlang._symboltable.CarLangLanguage;
import de.monticore.lang.montisim.carlang._symboltable.CarSymbol;
import de.monticore.lang.montisim.simlang._ast.ASTExplicitVehicle;
import de.monticore.lang.montisim.simlang._ast.ASTSimLangCompilationUnit;
import de.monticore.lang.montisim.simlang._parser.SimLangParser;
import de.monticore.lang.montisim.simlang._symboltable.*;
import de.monticore.lang.montisim.simlang._visitor.SimLangInheritanceVisitor;
import de.monticore.lang.montisim.simlang._visitor.SimLangVisitor;
import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import de.monticore.modelloader.ModelingLanguageModelLoader;
import de.monticore.symboltable.*;
import de.monticore.symboltable.resolving.AdaptedResolvingFilter;
import de.monticore.symboltable.resolving.CommonAdaptedResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingInfo;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.Map;
import java.util.Optional;

public class CarModelTest {

    public static void main(String[] args) throws IOException {
        new CarModelTest().run4();
    }

    public void run() throws IOException {
        /*SimLangContainer simLangContainer = SimLangTool.parseIntoContainer(
                getClass().getClassLoader().getResource("test/car/CarModelTest.sim").getPath(),
                getClass().getClassLoader().getResource("test/car/CMTCar.car").getPath()
        );
        System.out.println(simLangContainer.getExplicitVehicles().get().get(0).getCarModel().get());*/

        ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(new CarLangLanguage("CarLang", "car"){});
        family.addModelingLanguage(new SimLangLanguage("SimLang", "sim"){

        });
        // family.addModelingLanguage(new SimLangLang());

        // Warum CarSymbol.class und nicht CarModelSymbol.class??
        AdaptedResolvingFilter car2simFilter = new CommonAdaptedResolvingFilter<CarSymbol>(CarSymbol.KIND, CarSymbol.class, CarModelSymbol.KIND) {
            @Override
            public Symbol translate(Symbol symbol) {
                CarSymbol s = (CarSymbol) symbol;
                CarModelSymbol cms = new CarModelSymbol(s.getName());
                cms.setCarModel(s.getCarModel());
                return cms;
            }
        };
        family.addResolver(car2simFilter);

        ModelPath modelPath = new ModelPath(Paths.get("SimLang/src/test/resources/test/car"));

        Scope globalScope = new GlobalScope(modelPath, family);

        //Optional<CarSymbol> cmtCar = globalScope.resolve("CMTCar", CarSymbol.KIND);
        Optional<SimulationSymbol> cmt = globalScope.resolve("CarModelTest", SimulationSymbol.KIND);

        /*Optional<? extends ASTNode> sim = family.getLanguageByFileExtension("sim").get().getParser().parse(Paths.get("SimLang/src/test/resources/test/car/CarModelTest.sim").toAbsolutePath().toString());
        if (sim.isPresent()) {
            System.out.println("that worked");
        }*/

        ResolvingConfiguration rconf = new ResolvingConfiguration();
        rconf.addDefaultFilters(family.getAllResolvers());
        family.getLanguageByFileExtension("sim").get().getModelLoader().loadModelsIntoScope("CarModelTest", modelPath, (MutableScope) globalScope, rconf);

        // The following works, why does the first one not (i.e. why is the model not automatically loaded?)
        Optional<SimulationSymbol> cmt2 = globalScope.resolve("CarModelTest", SimulationSymbol.KIND);

        System.out.println("Encl scope");
        System.out.println(cmt2.get().getEnclosingScope().getEnclosingScope());

        System.out.println(globalScope.getSubScopes());
        System.out.println(globalScope.getLocalSymbols());

        /*if (cmtCar.isPresent()) {
            System.out.println("car present");
            CoCoCheckerFactory.getChecker().checkAll(cmtCar.get().getCarNode().get());
        } else {
            System.out.println("car not present");
        }*/

        if (cmt.isPresent()) {
            System.out.println("sim present");
        } else {
            System.out.println("sim not present");
        }

        if (cmt2.isPresent()) {
            System.out.println("sim2 present");
        } else {
            System.out.println("sim2 not present");
        }


        /*Optional<CarModelSymbol> cmtCar1 = globalScope.resolve("CMTCar", CarModelSymbol.KIND);
        if (cmtCar1.isPresent()) {
            // funktioniert, aber ich glaube der findet nicht das symbol im SimFile sondern im CarFile
            System.out.println(cmtCar1.get().getCarModel().getMass());
            System.out.println(cmtCar1.get().getEnclosingScope());
        } else {
            System.out.println("blub");
        }*/

        SimLangVisitor visitor = new SimLangInheritanceVisitor() {
            @Override
            public void handle(ASTExplicitVehicle node) {
                Optional<? extends Symbol> symbol = node.getCarModel().getSymbolOpt();
                if (symbol.isPresent()) {
                    CarModelSymbol s = (CarModelSymbol) symbol.get();
                    Optional<Symbol> carfilecarsymbol = s.getEnclosingScope().resolve(s.getName(), CarSymbol.KIND);
                    if (carfilecarsymbol.isPresent()) {
                        // Das funktioniert, ist das legitim?
                        System.out.println(((CarSymbol)carfilecarsymbol.get()).getCarModel().getMass());
                    }
                    // System.out.println(s.getCarModel().getMass());
                }
            }
        };
        visitor.handle(cmt2.get().getSimulationNode().get());

    }

    public void run2() {
        ASTSimLangCompilationUnit sim = SimLangTool.parse(Paths.get("SimLang/src/test/resources/test/car/CarModelTest.sim").toAbsolutePath().toString());
        SimLangTool.checkDefaultCoCos(sim);
        Scope symbolTable = SimLangTool.createSymbolTable(new SimLangLang(), sim);
        Optional<Symbol> cmt = symbolTable.resolve("CarModelTest", SimulationSymbol.KIND);
        if (cmt.isPresent()) {
            System.out.println("yay");
        } else {
            System.out.println("nay");
        }
    }

    public void run3() {
        ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(new CarLangLanguage("CarLang", "car"){});
        family.addModelingLanguage(new SimLangLang());

        // Warum CarSymbol.class und nicht CarModelSymbol.class??
        AdaptedResolvingFilter car2simFilter = new CommonAdaptedResolvingFilter<CarSymbol>(CarSymbol.KIND, CarSymbol.class, CarModelSymbol.KIND) {
            @Override
            public Symbol translate(Symbol symbol) {
                CarSymbol s = (CarSymbol) symbol;
                CarModelSymbol cms = new CarModelSymbol(s.getName());
                cms.setCarModel(s.getCarModel()); // This probably sets it to null
                return cms;
            }
        };
        family.addResolver(car2simFilter);

        ModelPath modelPath = new ModelPath(Paths.get("SimLang/src/test/resources/test/car"));
        Scope globalScope = new GlobalScope(modelPath, family);

        // Dieses resolve liefert ein leeres Optional, deswegen die nächste Zeile
        globalScope.resolve("CarModelTest.CarModelTest", SimulationSymbol.KIND);
        Optional<SimulationSymbol> carModelTest = globalScope.resolve("CarModelTest", SimulationSymbol.KIND);

        Optional<CarModelSymbol> cmtCar = globalScope.resolve("CMTCar", CarModelSymbol.KIND);
        System.out.println(cmtCar.isPresent());

        System.out.println(cmtCar.get().getFullName());
        System.out.println(cmtCar.get().getCarModel().getMass());

        SimLangVisitor visitor = new SimLangInheritanceVisitor() {
            @Override
            public void handle(ASTExplicitVehicle node) {
                Optional<? extends Symbol> symbol = node.getCarModel().getSymbolOpt();
                if (symbol.isPresent()) {
                    CarModelSymbolReference s = (CarModelSymbolReference) symbol.get();
                    System.out.println(s.getReferencedSymbol().getCarModel().getMass()); // <- works
                }
            }
        };
        visitor.handle(carModelTest.get().getSimulationNode().get());
    }

    public void run4() {
        SimLangContainer carModelTest = SimLangTool.parseIntoContainer(Paths.get("SimLang/src/test/resources/test/car"), "CarModelTest");
        System.out.println(carModelTest.getMapName().get());
    }

}
