/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GeneratorParamter extends DynamicParameter {
    public Double min;
    public Double max;
    public Double step;
    public Integer count;

    public GeneratorParamter() {
    }

    public GeneratorParamter(Double min) {
        this.min = min;
    }

    public GeneratorParamter(Double min, Double max, Double step) {
        this.min = min;
        this.max = max;
        this.step = step;
    }

    public GeneratorParamter(Double min, Double max, Integer count) {
        this.min = min;
        this.max = max;
        this.count = count;
    }

    public GeneratorParamter(Double min, Double max, Double step, Integer count) {
        this.min = min;
        this.max = max;
        this.step = step;
        this.count = count;
    }

    public List<Integer> getAllAsInt(){
        List<Double> asDouble = getAll();
        List<Integer> res = new ArrayList<>();
        asDouble.forEach(d -> res.add(d.intValue()));
        return res;
    }


    public List<Double> getAll(){
        if(!isValid()){
            return new ArrayList<>();
        }
        if(isSingleValue()){
            return Arrays.asList(min);
        }

        // min + (count - 1)*(mix - min)/(count-1)
        if(count != null && count > 1){
            List<Double> res = new ArrayList<>();
            double d = (max - min) / (count - 1);
            for(int i = 0; i < count; i++){
                res.add(min + i*d);
            }
            return res;
        }

        if(step == null){
            step = 1.0;
        }

        List<Double> res = new ArrayList<>();
        Double cur = min;
        while(max - cur > -0.000001){
            res.add(cur);
            cur += step;
        }
        return res;
    }

    public boolean isValid(){
        // only one value
        if(isSingleValue()){
            return true;
        }else
            //multiple values
            if(min != null && max != null){
                if(max < min){
                    return false;
                }else{
                    return (step != null  && step > 0) || (step == null) || (count != null && count > 1);
                }
            }else{
                return false;
            }
    }

    private boolean isSingleValue() {
        return min != null && max == null && step == null && count == null;
    }


}
