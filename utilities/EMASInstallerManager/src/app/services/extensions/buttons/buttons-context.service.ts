/* (c) https://github.com/MontiCore/monticore */
import { ButtonState } from "@services/extensions";

import * as path from "path";

export class ButtonContext {
    protected readonly state: ButtonState;

    public constructor(state: ButtonState) {
        this.state = state;
    }

    public get label(): string {
        return this.state.label;
    }

    public get image(): File {
        return this.state.image!;
    }

    public get script(): string {
        const script = this.state.script;

        if (script.endsWith(".bat")) return script.replace(".bat", '');
        else if (script.endsWith(".sh")) return script.replace(".sh", '');
        else return script;
    }

    public getScriptAsConstant(): string {
        return this.script.replace(/\./g, '_').toUpperCase();
    }

    public getScriptAsName(): string {
        const parts = this.script.split('.');
        const length = parts.length;

        for (let i = 0; i < length; i++) {
            const part = parts[i];

            parts[i] = part.charAt(0).toUpperCase() + part.substring(1);
        }

        return parts.join('');
    }

    public getImageAsName(): string {
        return path.basename(this.image.path).toLowerCase();
    }

    public getImageAsIdentifier(): string {
        const baseName = path.basename(this.image.path);
        const baseParts = baseName.split('.');

        baseParts.pop();

        return baseParts.join('-').toLowerCase();
    }

    public getImageAsConstant(): string {
        return this.getImageAsIdentifier().replace(/-/g, '_').toUpperCase();
    }
}
