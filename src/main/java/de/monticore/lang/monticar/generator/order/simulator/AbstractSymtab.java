/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.monticar.generator.order.simulator;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.java.lang.JavaDSLLanguage;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Common methods for symboltable tests
 *
 * @author Robert Heim
 */
public class AbstractSymtab {
    public static Scope createSymTab(String... modelPath) {
        ConstantPortSymbol.resetLastID();
        MathConverter.resetIDs();
        ThreadingOptimizer.resetID();
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        EmbeddedMontiArcMathLanguage montiArcLanguage = new EmbeddedMontiArcMathLanguage();

        TagMinMaxTagSchema.registerTagTypes(montiArcLanguage);
        TagTableTagSchema.registerTagTypes(montiArcLanguage);
        TagBreakpointsTagSchema.registerTagTypes(montiArcLanguage);
        TagExecutionOrderTagSchema.registerTagTypes(montiArcLanguage);
        TagInitTagSchema.registerTagTypes(montiArcLanguage);
        TagThresholdTagSchema.registerTagTypes(montiArcLanguage);
        TagDelayTagSchema.registerTagTypes(montiArcLanguage);

        fam.addModelingLanguage(montiArcLanguage);
        fam.addModelingLanguage(new StreamUnitsLanguage());
        // TODO should we use JavaDSLLanguage or add the resolvers in MALang?
        fam.addModelingLanguage(new JavaDSLLanguage());
        // TODO how to add java default types?
        Path argument;
        if (AbstractSymtab.class.getClassLoader().getResource("").getPath().contains(":")) {
            argument = Paths.get(AbstractSymtab.class.getClassLoader().getResource("").getPath().substring(1));
        } else {
            argument = Paths.get(AbstractSymtab.class.getClassLoader().getResource("").getPath());
        }
        final ModelPath mp = new ModelPath(Paths.get(argument + "/../../src/main/resources/defaultTypes"));
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        LogConfig.init();//TODO comment for debug output
        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }
}
