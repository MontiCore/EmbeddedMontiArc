/* (c) https://github.com/MontiCore/monticore */
import { ExtensionStateTransformation } from "@services/extensions/extensions-state-transformer";
import { ButtonState } from "@services/extensions";

import * as path from "path";
import * as fs from "fs-extra";

export class ButtonStateTransformation {
    protected readonly parent: ExtensionStateTransformation;
    protected readonly state: ButtonState;

    public constructor(parent: ExtensionStateTransformation, state: ButtonState) {
        this.parent = parent;
        this.state = state;
    }

    protected get image(): File {
        return this.state.image!;
    }

    protected getImageName(): string {
        return path.basename(this.image.path).toLowerCase();
    }

    protected async getImagesFolder(): Promise<string> {
        const rootFolder = await this.parent.getRootFolder();

        return path.join(rootFolder, "src", "browser", "images");
    }

    public async apply(): Promise<void> {
        const imagesFolder = await this.getImagesFolder();
        const imageName = this.getImageName();
        const destination = path.join(imagesFolder, imageName);

        return fs.copy(this.image.path, destination, { overwrite: true });
    }
}
