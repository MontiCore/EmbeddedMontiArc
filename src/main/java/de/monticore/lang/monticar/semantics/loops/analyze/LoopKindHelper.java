/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

public class LoopKindHelper {

    public static LoopKind combineKinds(LoopKind... kinds) {
        LoopKind res = LoopKind.Default;
        for (LoopKind kind : kinds) {
            res = combine(res, kind);
        }
        return res;
    }

    public static LoopKind combine(LoopKind kind1, LoopKind kind2) {
        switch (kind1) {
            case Linear:
                switch (kind2) {
                    case Polynom: return LoopKind.Polynom;
                    case NonLinear: return LoopKind.NonLinear;
                    case ODE: return LoopKind.ODE;
                    case DAE: return LoopKind.DAE;
                    default: return kind1;
                }
            case Polynom:
                switch (kind2) {
                    case NonLinear: return LoopKind.NonLinear;
                    case ODE: return LoopKind.DAE;
                    case DAE: return LoopKind.DAE;
                    default: return kind1;
                }
            case NonLinear:
                switch (kind2) {
                    case ODE: return LoopKind.DAE;
                    case DAE: return LoopKind.DAE;
                    default: return kind1;
                }
            case ODE:
                switch (kind2) {
                    case DAE: return LoopKind.DAE;
                    default: return kind1;
                }
            case DAE:
                return LoopKind.DAE;
            case Default:
                return kind2;
            default: return kind1;
        }
    }
}
