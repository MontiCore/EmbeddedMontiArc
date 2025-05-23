import os
import shutil

def replace_job_token_in_settings(file_path):
    """
    Ersetzt 'Job-Token' durch 'Private-Token' in der settings.xml-Datei.
    :param file_path: Pfad zur settings.xml-Datei
    """
    with open(file_path, 'r', errors='ignore') as file:
        content = file.read()

    # Überprüfen, ob 'Job-Token' vorhanden ist
    if 'Job-Token' in content:
        # Backup der Originaldatei erstellen
        backup_path = file_path + '.bak'
        shutil.copy(file_path, backup_path)
        print(f"Backup erstellt: {backup_path}")

        # 'Job-Token' durch 'Private-Token' ersetzen
        updated_content = content.replace('Job-Token', 'Private-Token')

        # Datei mit den Änderungen speichern
        with open(file_path, 'w') as file:
            file.write(updated_content)
        print(f"'Job-Token' ersetzt in Datei: {file_path}")
    else:
        print(f"Kein 'Job-Token' gefunden in: {file_path}")

def process_settings_files(repo_path):
    """
    Durchsucht das Repository nach settings.xml-Dateien und ersetzt 'Job-Token'.
    :param repo_path: Pfad zum Repository
    """
    for root, _, files in os.walk(repo_path):
        for file in files:
            if file == 'settings.xml':
                file_path = os.path.join(root, file)
                replace_job_token_in_settings(file_path)

if __name__ == "__main__":
    repo_path = '../repos/MNISTCalculator'  # Pfad zum Repository
    process_settings_files(repo_path)