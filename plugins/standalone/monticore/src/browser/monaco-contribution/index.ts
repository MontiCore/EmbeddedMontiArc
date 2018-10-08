/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { MONTICORE_LANGUAGE_ID, MONTICORE_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./monticore-monaco-language";

monaco.languages.register({
    id: MONTICORE_LANGUAGE_ID,
    extensions: [".mc4"],
    aliases: [MONTICORE_LANGUAGE_NAME],
    mimetypes: ["text/x-monticore"]
});

monaco.languages.onLanguage(MONTICORE_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(MONTICORE_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(MONTICORE_LANGUAGE_ID, monarchLanguage);
});
