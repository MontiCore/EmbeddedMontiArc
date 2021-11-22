/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema OverrideWithPropertyInSuperSchema extends OverrideWithPropertyInSuperSchemaSuper {

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