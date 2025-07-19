/* (c) https://github.com/MontiCore/monticore */
import { ExtensionState } from "@services/extensions/extensions-state.service";
import { ButtonContext } from "@services/extensions/buttons/buttons-context.service";
import { Injectable } from "@angular/core";

export class ExtensionContext {
    protected readonly state: ExtensionState;
    protected readonly buttons: ButtonContext[];

    public constructor(state: ExtensionState) {
        this.state = state;
        this.buttons = this.state.getButtons().map(button => new ButtonContext(button));
    }

    public get name(): string {
        return this.state.name;
    }

    public getButtons(): ButtonContext[] {
        return this.buttons;
    }

    public getButtonsWithUniqueImage(): ButtonContext[] {
        const buttons = [];
        const identifiers = new Set<string>();

        for (const button of this.buttons) {
            const identifier = button.getImageAsIdentifier();

            if (!identifiers.has(identifier)) {
                buttons.push(button);
                identifiers.add(identifier);
            }
        }

        return buttons;
    }
}

@Injectable({ providedIn: "root" })
export class ExtensionsContextService {
    public createContext(state: ExtensionState): ExtensionContext {
        return new ExtensionContext(state);
    }
}
