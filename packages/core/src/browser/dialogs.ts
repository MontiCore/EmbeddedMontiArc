/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { SingleTextInputDialog, SingleTextInputDialogProps } from "@theia/core/lib/browser";
import { Message } from "@phosphor/messaging";

@injectable()
export class AsyncSingleTextInputDialogProps extends SingleTextInputDialogProps {
    public readonly validateAsync?: (input: string) => Promise<string>;
}

export class AsyncSingleTextInputDialog extends SingleTextInputDialog {
    public constructor(
        @inject(AsyncSingleTextInputDialogProps) protected readonly props: AsyncSingleTextInputDialogProps
    ) {
        super(props);
    }

    protected async onUpdateRequest(message: Message): Promise<void> {
        super.onUpdateRequest(message);

        if (this.resolve) {
            const value = this.value;
            const error = await this.isValidAsync(value);

            this.setErrorMessage(error);
        }
    }

    public async isValidAsync(value: string): Promise<string> {
        const validity = this.isValid(value);
        const asyncValidity = this.props.validateAsync ? await this.props.validateAsync(value) : '';

        return validity.length > 0 ? validity : asyncValidity;
    }
}
