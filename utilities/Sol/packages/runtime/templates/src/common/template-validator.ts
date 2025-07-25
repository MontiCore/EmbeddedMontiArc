/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonValidator } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { injectable } from "inversify";
import { TEMPLATE_VALIDATOR_TYPE } from "./templates-protocol";

@injectable()
export abstract class CommonTemplateValidator<V, E> extends CommonValidator<V, E> {
    protected constructor(id: string) {
        super(id, TEMPLATE_VALIDATOR_TYPE);
    }
}
