/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { EmamComponentFrontendValidatorTOP } from "./emamcomponent-frontend-validator-top";

import { EmamComponentOptions } from "../../common/validators/emamcomponent-protocol";

@injectable()
export class EmamComponentFrontendValidator extends EmamComponentFrontendValidatorTOP {
    public async validatePortsDirection(direction: string, options: EmamComponentOptions): Promise<string | undefined> {
        if (direction !== "in" && direction !== "out") return "A port's direction can only be 'in' or 'out'.";
    }

    public async validatePortsType(type: string, options: EmamComponentOptions): Promise<string | undefined> {
        if (type !== "N" && type !== "Q" && type !== "Z") return "A port's type can only be a mathemtical set.";
    }
}
