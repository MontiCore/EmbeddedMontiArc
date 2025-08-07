/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema OverriddenComplexProperty extends ComplexProperty {

    override strategy {
        values:
            ornstein_uhlenbeck,
            gaussian;

        define ornstein_uhlenbeck {
            mu: Q*
            theta: Q*
            sigma: Q*
        }

        define gaussian {
            noise_variance: Q
        }
    }
}