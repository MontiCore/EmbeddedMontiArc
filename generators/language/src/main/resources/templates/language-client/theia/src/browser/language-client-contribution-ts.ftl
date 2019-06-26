/*
 * Copyright (C) ${year} SE RWTH.
 *
 * ${license}
 */

import { BaseLanguageClientContribution } from "@theia/languages/lib/browser";
import { injectable } from "inversify";
import { ${Grammar}Language } from "../common";

@injectable()
export class ${Grammar}ClientContribution extends BaseLanguageClientContribution {
    public readonly id: string = ${Grammar}Language.ID;
    public readonly name: string = ${Grammar}Language.NAME;

    protected get globPatterns(): string[] {
        return ["**/*.car"];
    }
}
