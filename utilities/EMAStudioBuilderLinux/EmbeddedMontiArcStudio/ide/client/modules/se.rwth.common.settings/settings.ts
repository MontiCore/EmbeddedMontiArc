/* (c) https://github.com/MontiCore/monticore */
export class Settings {
    protected settings: any;

    protected constructor() {
        this.doLoad();
    }

    private static instance: Settings;

    protected static getInstance(): Settings {
        return Settings.instance = Settings.instance ? Settings.instance : new Settings();
    }

    public static load(): void {
        return Settings.getInstance().doLoad();
    }

    protected static readonly KEY = "settings";

    protected doLoad(): void {
        const data = window.localStorage.getItem(Settings.KEY) || "{}";

        this.settings = JSON.parse(data);
    }

    public static save(): void {
        Settings.getInstance().doSave();
    }

    protected doSave(): void {
        const data = JSON.stringify(this.settings);

        window.localStorage.setItem(Settings.KEY, data);
    }

    public static set(key: string, value: any): void {
        Settings.getInstance().doSet(key, value);
    }

    protected doSet(key: string, value: any): void {
        this.settings[key] = value;

        this.doSave();
    }

    public static get(key: string): any {
        return Settings.getInstance().doGet(key);
    }

    protected doGet(key: string): any {
        return this.settings[key];
    }
}
