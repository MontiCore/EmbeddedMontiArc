/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { TAGSCHEMA_LANGUAGE_ID, TAGSCHEMA_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./tagschema-monaco-language";

monaco.languages.register({
    id: TAGSCHEMA_LANGUAGE_ID,
    extensions: [".tagschema"],
    aliases: [TAGSCHEMA_LANGUAGE_NAME],
    mimetypes: ["text/x-tagschema"]
});

monaco.languages.onLanguage(TAGSCHEMA_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(TAGSCHEMA_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(TAGSCHEMA_LANGUAGE_ID, monarchLanguage);
});
