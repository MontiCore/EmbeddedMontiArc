/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.qualityMetric;

import de.se_rwth.commons.logging.Log;

public enum MetricType {
    CommunicationCost,
    Silhouette;

    public Metric toMetric(){
        switch (this){
            case CommunicationCost: return new CommunicationCostMetric();
            case Silhouette: return new SilhouetteMetric();
            default:{
                Log.error("Unknown metric: " + this);
                return null;
            }
        }
    }

}
