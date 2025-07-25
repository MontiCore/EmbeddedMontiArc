/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { GeneratorService } from "@services/common/generator.service";
import { UseCaseState } from "@services/use-cases/use-cases-state.service";
import { UseCaseContext, UseCasesContextService } from "@services/use-cases/use-cases-context.service";

@Injectable({ providedIn: "root" })
export class UseCaseGeneratorService {
    public constructor(
        protected readonly generator: GeneratorService,
        protected readonly contexts: UseCasesContextService
    ) {}

    public async generate(root: string, state: UseCaseState): Promise<void> {
        const context = this.contexts.createContext(state);

        this.generator.configure("use-case");

        await Promise.all([
            this.generateReadme(root, context),
            this.generateConfigs(root, context)
        ])
    }

    protected async generateReadme(root: string, context: UseCaseContext): Promise<void> {
        return this.generator.generate(root, "README.md.njk", context);
    }

    protected async generateConfigs(root: string, context: UseCaseContext): Promise<void> {
        return this.generator.generate(root, ".elysium/workspace-initiator/configs.json.njk", context);
    }
}
