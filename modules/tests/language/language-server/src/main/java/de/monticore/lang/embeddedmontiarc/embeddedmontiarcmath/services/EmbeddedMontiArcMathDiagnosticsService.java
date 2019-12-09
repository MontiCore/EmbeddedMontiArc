/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.services;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls.FindingToDiagnostic;

@Singleton
public class EmbeddedMontiArcMathDiagnosticsService extends EmbeddedMontiArcMathDiagnosticsServiceTOP {
    protected final FluentLogger logger;

    @Inject
    protected EmbeddedMontiArcMathDiagnosticsService(FindingToDiagnostic f2d) {
        super(f2d);

        this.logger = FluentLogger.forEnclosingClass();
    }
}
