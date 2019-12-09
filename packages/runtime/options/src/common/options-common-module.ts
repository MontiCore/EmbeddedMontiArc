/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider } from "@theia/core";
import { interfaces } from "inversify";
import { ValidatorContribution } from "./validator-registry";

import Bind = interfaces.Bind;

export function bindCommon(bind: Bind) {
    bindContributionProvider(bind, ValidatorContribution);
}
