/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.CommonScopeSpanningSymbol;

import java.util.Optional;

public class ConfigurationSymbol extends CommonScopeSpanningSymbol {

    private NumEpochSymbol numEpoch;
    private BatchSizeSymbol batchSize;
    private LoadCheckpointSymbol loadCheckpoint;
    private NormalizeSymbol normalize;
    private OptimizerSymbol optimizer;

    public static final ConfigurationSymbolKind KIND = new ConfigurationSymbolKind();

    public ConfigurationSymbol()  {
        super("", KIND);
    }

    public NumEpochSymbol getNumEpoch() {
        return numEpoch;
    }

    public void setNumEpoch(NumEpochSymbol numEpoch) {
        this.numEpoch = numEpoch;
    }

    public OptimizerSymbol getOptimizer() {
        return optimizer;
    }

    public void setOptimizer(OptimizerSymbol optimizer) {
        this.optimizer = optimizer;
    }

    public BatchSizeSymbol getBatchSize() {
        return batchSize;
    }

    public void setBatchSize(BatchSizeSymbol batchSize) {
        this.batchSize = batchSize;
    }

    public LoadCheckpointSymbol getLoadCheckpoint() {
        return loadCheckpoint;
    }

    public void setLoadCheckpoint(LoadCheckpointSymbol loadCheckpoint) {
        this.loadCheckpoint = loadCheckpoint;
    }

    public NormalizeSymbol getNormalize() {
        return normalize;
    }

    public void setNormalize(NormalizeSymbol normalize) {
        this.normalize = normalize;
    }


}
