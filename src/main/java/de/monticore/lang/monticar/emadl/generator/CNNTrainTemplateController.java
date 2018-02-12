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
package de.monticore.lang.monticar.emadl.generator;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;

import java.util.Optional;

public class CNNTrainTemplateController {

    //todo: support only LinearRegression(squared loss), LogisticRegression and Softmax(cross entropy loss) as loss function

    private ConfigurationSymbol configuration;

    public CNNTrainTemplateController(ConfigurationSymbol configuration) {
        this.configuration = configuration;
    }

    public boolean isCpp(){
        return false;
    }

    public boolean isPython(){
        return true;
    }

    public String getLrPolicy(){
        return null;
    }

    public Optional<String> getLearningRate(){
        return null;
    }

    public Optional<String> getRescaleGrad(){
        return null;
    }

    public Optional<String> getClipGradient(){
        return null;
    }

    public Optional<String> getMomentum(){
        return null;
    }

    public Optional<String> getBeta1(){
        return null;
    }

    public Optional<String> getBeta2(){
        return null;
    }

    public Optional<String> getEpsilon(){
        return null;
    }

    public Optional<String> getEps(){
        return null;
    }

    public Optional<String> getGamma1(){
        return null;
    }

    public Optional<String> getGamma2(){
        return null;
    }

    public Optional<String> getCentered(){
        return null;
    }

    public Optional<String> getClipWeights(){
        return null;
    }

    public Optional<String> getRho(){
        return null;
    }

    public Optional<String> getLearningRateDecay(){
        return null;
    }

    public Optional<String> getStepSize(){
        return null;
    }

    public Optional<String> getStepList(){
        return null;
    }

    public Optional<String> getNumEpochs(){
        return null;
    }

    public Optional<String> getEvalMetric(){
        return null;
    }

    public Optional<String> getCheckpoint(){
        return null;
    }

}