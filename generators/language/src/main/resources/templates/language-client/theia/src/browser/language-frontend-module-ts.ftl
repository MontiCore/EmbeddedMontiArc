/*
 * Copyright (C) ${year} SE RWTH.
 */

import { LanguageClientContribution } from "@theia/languages/lib/browser";
import { ContainerModule } from "inversify";
import { ${Grammar}ClientContribution } from "./${grammar}-client-contribution";
import { ${Grammar}GrammarContribution } from "./${grammar}-grammar-contribution";
import { LanguageGrammarDefinitionContribution } from "@theia/monaco/lib/browser/textmate";

export default new ContainerModule(bind => {
    bind(${Grammar}GrammarContribution).toSelf().inSingletonScope();
    bind(LanguageGrammarDefinitionContribution).to(${Grammar}GrammarContribution).inSingletonScope();

    bind(${Grammar}ClientContribution).toSelf().inSingletonScope();
    bind(LanguageClientContribution).toService(${Grammar}ClientContribution);
});
