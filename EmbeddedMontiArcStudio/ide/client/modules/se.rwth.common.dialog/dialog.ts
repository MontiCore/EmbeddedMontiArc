/* (c) https://github.com/MontiCore/monticore */
import {UIModule} from "../se.rwth.common.module/module";

export class Dialog extends UIModule {
    protected $container: JQuery;
    protected $message: JQuery;
    protected $icon: JQuery;
    protected $panel: JQuery;

    protected constructor() {
        super("common.dialog");
    }

    protected setDependencies(): void {}

    protected setHTMLElements(): void {
        this.$container = jQuery("#dialog-container");
        this.$message = jQuery("#dialog-message");
        this.$icon = jQuery("#dialog-icon");
        this.$panel = jQuery("#dialog-panel");
    }

    protected addEventListeners(): void {}

    protected static instance: Dialog;

    public static create(): Dialog {
        return Dialog.instance = Dialog.instance ? Dialog.instance : new Dialog();
    }


    public show(message: string, type: string = "error", buttons: string[] = []): void {
        this.removeButtons();
        this.message(message);
        this.type(type);
        this.addButtons(buttons);
        this.$container.show();
    }

    public hide(): void {
        this.$container.hide();
    }

    public message(message?: string): void | string {
        return message ? this.setMessage(message) : this.getMessage();
    }

    public type(type?: string): void | string {
        return type ? this.setType(type) : this.getType();
    }


    private removeButtons(): void {
        this.$panel.children(".button").remove();
    }

    private static readonly RADIX: string = "modules/se.rwth.common.dialog/images/";

    private setType(type: string): void {
        const iconPath = Dialog.RADIX + (type === "error" ? "error.svg" : "warning.svg");
        const iconURL = "url(\"" + iconPath + "\")";

        this.$icon.css("background-image", iconURL);
    }

    private getType(): string {
        const backgroundImage = this.$icon.css("background-image");
        const isError = backgroundImage.endsWith("error.svg");

        return isError ? "error" : "warning";
    }

    private setMessage(message: string): void {
        this.$message.html(message);
    }

    private getMessage(): string {
        return this.$message.html();
    }

    private addButtons(labels: string[]): void {
        const length = labels.length;

        for(let i = 0; i < length; i++) {
            this.addButton(labels[i], i);
        }
    }

    private addButton(label: string, index: number): void {
        const $button = jQuery("<span/>", { "class": "button", text: label });

        const callback = () => {
            this.emit("click", index);
            this.hide();
        };

        $button.one("click", callback);
        this.$panel.append($button);
    }
}
