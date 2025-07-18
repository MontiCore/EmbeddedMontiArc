/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { MACOCOVIZ_LANGUAGE_ID, MACOCOVIZ_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./macocoviz-monaco-language";

monaco.languages.register({
    id: MACOCOVIZ_LANGUAGE_ID,
    extensions: [".viz"],
    aliases: [MACOCOVIZ_LANGUAGE_NAME],
    mimetypes: ["text/x-macocoviz"]
});

monaco.languages.onLanguage(MACOCOVIZ_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(MACOCOVIZ_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(MACOCOVIZ_LANGUAGE_ID, monarchLanguage);
});
