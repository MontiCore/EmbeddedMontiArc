import {UIModule} from "../se.rwth.common.module/module";

export class Loader extends UIModule {
    protected $container: JQuery;
    protected $message: JQuery;

    protected constructor() {
        super("common.loader");
    }

    protected setDependencies(): void {}

    protected setHTMLElements(): void {
        this.$container = jQuery("#loader-container");
        this.$message = jQuery("#loader-message");
    }

    protected addEventListeners(): void {}

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
}