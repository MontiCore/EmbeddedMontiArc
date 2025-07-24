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
                    case LinearDifferencial: return LoopKind.LinearDifferencial;
                    case NonLinearDifferencial: return LoopKind.NonLinearDifferencial;
                    default: return kind1;
                }
            case Polynom:
                switch (kind2) {
                    case NonLinear: return LoopKind.NonLinear;
                    case LinearDifferencial: return LoopKind.NonLinearDifferencial;
                    case NonLinearDifferencial: return LoopKind.NonLinearDifferencial;
                    default: return kind1;
                }
            case NonLinear:
                switch (kind2) {
                    case LinearDifferencial: return LoopKind.NonLinearDifferencial;
                    case NonLinearDifferencial: return LoopKind.NonLinearDifferencial;
                    default: return kind1;
                }
            case LinearDifferencial:
                switch (kind2) {
                    case NonLinearDifferencial: return LoopKind.NonLinearDifferencial;
                    default: return kind1;
                }
            case NonLinearDifferencial:
                return LoopKind.NonLinearDifferencial;
            case Default:
                return kind2;
            default: return kind1;
        }
    }
}
