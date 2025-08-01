/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonValidator } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { injectable, unmanaged } from "inversify";
import { MODULE_VALIDATOR_TYPE } from "./module-protocol";

@injectable()
export abstract class CommonModuleValidator extends CommonValidator<string, string | true> {
    protected constructor(@unmanaged() id: string) {
        super(id, MODULE_VALIDATOR_TYPE);
    }
}
