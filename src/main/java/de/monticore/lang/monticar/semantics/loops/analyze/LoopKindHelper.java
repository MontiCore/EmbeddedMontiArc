/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import static de.monticore.lang.monticar.semantics.loops.analyze.LoopKind.*;

public class LoopKindHelper {

    public static LoopKind combineKinds(LoopKind... kinds) {
        LoopKind res = LoopKind.Constant;
        for (LoopKind kind : kinds) {
            res = combine(res, kind);
        }
        return res;
    }

    public static LoopKind combine(LoopKind kind1, LoopKind kind2) {
        switch (kind1) {
            case Linear:
                switch (kind2) {
                    case Polynom: return Polynom;
                    case NonLinear: return NonLinear;
                    case ODE: return DAE;
                    case DAE: return DAE;
                    default: return Linear;
                }
            case Polynom:
                switch (kind2) {
                    case NonLinear: return NonLinear;
                    case ODE: return DAE;
                    case DAE: return DAE;
                    default: return Polynom;
                }
            case NonLinear:
                switch (kind2) {
                    case ODE: return DAE;
                    case DAE: return DAE;
                    default: return NonLinear;
                }
            case ODE:
                switch (kind2) {
                    case ODE: return ODE;
                    case Constant: return ODE;
                    default: return DAE;
                }
            case DAE:
                return DAE;
            case Constant:
                return kind2;
            default: return kind1;
        }
    }
}
