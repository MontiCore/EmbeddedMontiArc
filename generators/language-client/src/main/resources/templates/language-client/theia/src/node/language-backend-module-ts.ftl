/*
 * Copyright (C) ${year} SE RWTH.
 */

import { LanguageServerContribution } from "@theia/languages/lib/node";
import { ContainerModule } from "inversify";
import { ${Grammar}ServerContribution } from "./${grammar}-server-contribution";

export default new ContainerModule(bind => {
    bind(${Grammar}ServerContribution).toSelf().inSingletonScope();
    bind(LanguageServerContribution).toService(${Grammar}ServerContribution);
});
