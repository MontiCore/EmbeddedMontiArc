/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { GeneratorService } from "@services/common/generator.service";
import { ExtensionState } from "@services/extensions/extensions-state.service";
import { ExtensionContext, ExtensionsContextService } from "@services/extensions/extensions-context.service";

@Injectable({ providedIn: "root" })
export class ExtensionGeneratorService {
    public constructor(
        protected readonly generator: GeneratorService,
        protected readonly contexts: ExtensionsContextService
    ) {}

    public async generate(root: string, state: ExtensionState): Promise<void> {
        const context = this.contexts.createContext(state);

        this.generator.configure("extension");

        await Promise.all([
            this.generateIndexCSS(root, context),
            this.generateBrowserIndexTS(root, context),
            this.generateCommandContribution(root, context),
            this.generateConditions(root, context),
            this.generateFrontendModule(root, context),
            this.generateMenuContribution(root, context),
            this.generateCommonIndexTS(root, context),
            this.generateProtocol(root, context),
            this.generateCompileTSConfig(root, context),
            this.generatePackage(root, context),
            this.generateReadme(root, context)
        ]);
    }

    protected async generateIndexCSS(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/browser/style/index.css.njk", context);
    }

    protected async generateBrowserIndexTS(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/browser/index.ts.njk", context);
    }

    protected async generateCommandContribution(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/browser/{{name}}-command-contribution.ts.njk", context);
    }

    protected async generateConditions(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/browser/{{name}}-conditions.ts.njk", context);
    }

    protected async generateFrontendModule(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/browser/{{name}}-frontend-module.ts.njk", context);
    }

    protected async generateMenuContribution(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/browser/{{name}}-menu-contribution.ts.njk", context);
    }

    protected async generateCommonIndexTS(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/common/index.ts.njk", context);
    }

    protected async generateProtocol(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "src/common/{{name}}-protocol.ts.njk", context);
    }

    protected async generateCompileTSConfig(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "compile.tsconfig.json.njk", context);
    }

    protected async generatePackage(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "package.json.njk", context);
    }

    protected async generateReadme(root: string, context: ExtensionContext): Promise<void> {
        return this.generator.generate(root, "README.md.njk", context);
    }
}
