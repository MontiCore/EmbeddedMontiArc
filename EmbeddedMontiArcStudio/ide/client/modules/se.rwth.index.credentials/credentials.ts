/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class Credentials extends Module {
    protected $container: JQuery;
    protected $inputUsername: JQuery;
    protected $inputPassword: JQuery;
    protected $buttonSubmit: JQuery;
    protected $buttonCancel: JQuery;

    protected constructor() {
        super();

    }

    protected static instance: Credentials;

    public static async create(): Promise<Credentials> {

    }

    protected setHTMLElements(): void {
        this.$container = jQuery("#credentials-container");
        this.$inputUsername = jQuery("#credentials-username");
        this.$inputPassword = jQuery("#credentials-password");
        this.$buttonSubmit = jQuery("#credentials-submit");
        this.$buttonCancel = jQuery("#credentials-cancel");
    }

    protected addEventListeners(): void {

    }


    public show(): void {
        this.$container.show();
        this.emit("show");
    }

    public hide(): void {
        this.$container.hide();
        this.emit("hide");
    }

    public getUsername(): string {
        return this.$inputUsername.val() as string;
    }

    public getPassword(): string {
        return this.$inputPassword.val() as string;
    }

    public areAvailable(): boolean {
        const username = this.getUsername();
        const password = this.getPassword();

        return username.length > 0 && password.length > 0;
    }


    private onSubmitClick(): void {

    }

    private onCancelClick(): void {

    }
}
