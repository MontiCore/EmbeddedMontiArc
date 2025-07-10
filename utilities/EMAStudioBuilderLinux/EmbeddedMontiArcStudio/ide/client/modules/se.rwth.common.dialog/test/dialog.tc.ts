/* (c) https://github.com/MontiCore/monticore */
import {ClientFunction, Selector, t} from "testcafe";
import * as Util from "util";
import {Constants} from "../../se.rwth.common.test/constants";

declare var Dialog;

export class DialogTestController {
    protected $container: Selector;
    protected $message: Selector;
    protected $icon: Selector;
    protected $panel: Selector;
    protected $buttons: Selector;

    public constructor() {
        this.$container = Selector("#dialog-container");
        this.$message = Selector("#dialog-message");
        this.$icon = Selector("#dialog-icon");
        this.$panel = Selector("#dialog-panel");
        this.$buttons = this.$panel.child(".shared-span");
    }

    /*
     * Methods
     */
    public async show(message: string, type: string, buttons: string[], callbacks: Function[] = [], error?: any): Promise<void> {
        return ClientFunction((message, type, buttons, callbacks, error) => Dialog.show(message, type, buttons, callbacks, error)).apply(null, arguments);
    }

    public async hide(): Promise<void> {
        return ClientFunction(() => Dialog.hide()).apply(null, arguments);
    }

    /*
     * Checks
     */
    protected async checkVisibility(expectedVisibility: string, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const currentVisibility = this.$container.getStyleProperty("display");

        return t.expect(currentVisibility).eql(expectedVisibility, "[Dialog]: Visibility Violation", { timeout: timeout });
    }

    protected async checkIcon(type: string, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const icon = type === "error" ? "error.svg" : "warning.svg";
        const background = await this.$icon.getStyleProperty("background-image");
        const hasIcon = background.indexOf(icon) > -1;

        return t.expect(hasIcon).ok("[Dialog]: Type Violation", { timeout: timeout });
    }

    protected async checkMessage(message: string, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const span = this.$message.addCustomDOMProperties({ innerHTML: node => node.innerHTML });
        const html = await span["innerHTML"];

        return t.expect(html).eql(message, "[Dialog]: Message Violation", { timeout: timeout });
    }

    protected async checkButtons(labels: string[], timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const length = labels.length;

        for(let i = 0; i < length; i++) {
            const button = this.$buttons.nth(i);

            await t.expect(button.textContent).eql(labels[i], { timeout: timeout });
        }
    }

    public async checkShow(message: string, type: string, labels: string[], timeout?: number): Promise<void[]> {
        return Promise.all([
            this.checkVisibility("block", timeout),
            this.checkMessage(message),
            this.checkIcon(type),
            this.checkButtons(labels)
        ]);
    }

    public async checkHide(timeout?: number): Promise<void> {
        return this.checkVisibility("none", timeout);
    }

    /*
     * Actions
     */
    public async clickButton(index: number): Promise<void> {
        const button = this.$buttons.nth(index);
        const message = Util.format("[Dialog]: Button at Position %d has been clicked.", index);

        console.log(message);
        return t.click(button);
    }
}
