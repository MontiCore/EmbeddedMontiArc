/* (c) https://github.com/MontiCore/monticore */
import { Extension, ExtensionsService } from "@services/extensions/extensions.service";
import { ExtensionsStateService, ExtensionState } from "@services/extensions/extensions-state.service";
import { ButtonStateTransformation } from "@services/extensions/buttons/buttons-state-transformer.service";
import { ExtensionGeneratorService } from "@services/extensions/extension-generator.service";
import { Injectable } from "@angular/core";
import { TargetService } from "@services/common/target.service";

import * as path from "path";
import * as fs from "fs-extra";

export class ExtensionStateTransformation {
    protected readonly state: ExtensionState;
    protected readonly generator: ExtensionGeneratorService;
    protected readonly extensions: ExtensionsService;
    protected readonly target: TargetService;

    public constructor(state: ExtensionState, extensions: ExtensionsService,
                       generator: ExtensionGeneratorService, target: TargetService) {
        this.state = state;
        this.extensions = extensions;
        this.generator = generator;
        this.target = target;
    }

    protected get name(): string {
        return this.state.name.toLowerCase();
    }

    public async getRootFolder(): Promise<string> {
        const extensionsFolder = await this.extensions.getExtensionsFolder();

        return path.join(extensionsFolder, this.name);
    }

    protected async addToTarget(): Promise<void> {
        const packageFile = await this.target.getPackageFile();
        const packageJSON = await fs.readJSON(packageFile);
        const name = `@emastudio/${this.name}`;

        packageJSON.dependencies[name] = "^0.1.0";

        return fs.writeJSON(packageFile, packageJSON, { spaces: 4 });
    }

    public async apply(): Promise<Extension> {
        const root = await this.getRootFolder();
        const buttons = this.state.getButtons(true);

        await this.generator.generate(root, this.state);

        for (const button of buttons) {
            const transformation = new ButtonStateTransformation(this, button);

            await transformation.apply();

            button.saved = true;
        }

        await this.addToTarget();

        return { name: `@emastudio/${this.name}` } as Extension;
    }
}

@Injectable({ providedIn: "root" })
export class ExtensionsStateTransformer {
    public constructor(
        protected readonly extensions: ExtensionsService,
        protected readonly states: ExtensionsStateService,
        protected readonly generator: ExtensionGeneratorService,
        protected readonly target: TargetService
    ) {}

    public async apply(): Promise<void> {
        const states = this.states.getStates(true) as ExtensionState[];

        for (const state of states) {
            const transformation = new ExtensionStateTransformation(state, this.extensions, this.generator, this.target);
            const extension = await transformation.apply();

            state.saved = true;

            this.extensions.addExtension(extension);
            this.target.addDependency(extension);
        }
    }
}
