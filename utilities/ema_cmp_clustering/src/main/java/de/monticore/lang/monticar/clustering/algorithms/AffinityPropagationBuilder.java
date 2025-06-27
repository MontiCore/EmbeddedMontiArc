/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

public class AffinityPropagationBuilder {
    private Long seed;

    // parameter list, true if mandatory
    public enum AffinityPropagationParameters {
        SEED(false);
        private Boolean mandatory;

        AffinityPropagationParameters(Boolean mandatory) {
            this.mandatory = mandatory;
        }
        public Boolean isMandatory() {
            return this.mandatory;
        }
    }

    public AffinityPropagationBuilder() {
    }

    public AffinityPropagationBuilder(long seed) {
        this.seed = seed;
    }

    public void setSeed(long seed) {
        this.seed = seed;
    }

    public AffinityPropagationAlgorithm build() {
        AffinityPropagationAlgorithm affinityPropagationAlgorithm = new AffinityPropagationAlgorithm();
        if(seed == null){
            return affinityPropagationAlgorithm;
        }else{
            affinityPropagationAlgorithm.setArgs(new Object[]{AffinityPropagationParameters.SEED, seed});
            return affinityPropagationAlgorithm;
        }
    }

}
