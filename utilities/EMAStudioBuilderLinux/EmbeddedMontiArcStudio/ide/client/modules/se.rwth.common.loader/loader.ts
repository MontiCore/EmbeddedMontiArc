/* (c) https://github.com/MontiCore/monticore */
import {UIModule} from "../se.rwth.common.module/module";
import {Dialog} from "../se.rwth.common.dialog/dialog";

export class Loader extends UIModule {
    protected dialog: Dialog;

    protected $container: JQuery;
    protected $message: JQuery;

    protected constructor() {
        super("common.loader");
    }

    protected setDependencies(): void {
        this.dialog = Dialog.create();
    }

    protected setHTMLElements(): void {
        this.$container = jQuery("#loader-container");
        this.$message = jQuery("#loader-message");
    }

    protected addEventListeners(): void {
        this.dialog.on("show", this.onDialogShow);
    }

    protected static instance: Loader;

    public static create(): Loader {
        return Loader.instance = Loader.instance ? Loader.instance : new Loader();
    }


    public show(message: string): void {
        this.$message.html(message);
        this.$container.show();
        this.emit("show", message);
    }

    public hide(): void {
        this.$container.hide();
        this.emit("hide");
    }

    public message(message: string): void {
        this.$message.html(message);
        this.emit("message", message);
    }


    private onDialogShow(): void {
        this.hide();
    }
}
