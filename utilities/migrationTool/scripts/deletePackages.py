import requests
import yaml
from urllib.parse import quote

def delete_all_github_packages(token, package_type="container"):
    """
    Löscht alle GitHub-Pakete eines bestimmten Typs aus deinem Account.

    :param token: GitHub-Personal-Access-Token mit `delete:packages`-Berechtigung
    :param package_type: Typ der Pakete (z. B. 'container', 'npm', etc.)
    """
    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json"
    }

    try:
        # Abrufen aller Pakete eines bestimmten Typs
        response = requests.get(f"https://api.github.com/user/packages?package_type={package_type}", headers=headers)
        if response.status_code != 200:
            print(f"Fehler beim Abrufen der Pakete: {response.status_code} - {response.text}")
            return

        packages = response.json()
        for package in packages:
            package_name = package["name"]
            print(f"Lösche Paket: {package_name}")

            package_name_encoded = quote(package_name)
            delete_url = f"https://api.github.com/user/packages/{package_type}/{package_name_encoded}"
            delete_response = requests.delete(delete_url, headers=headers)
            if delete_response.status_code == 204:
                print(f"Paket {package_name} erfolgreich gelöscht.")
            else:
                print(f"Fehler beim Löschen von {package_name}: {delete_response.status_code} - {delete_response.text}")
    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

# Beispielaufruf
try:
    config = yaml.safe_load(open("../config.yaml"))
    github_token = config["TargetToken"]
    delete_all_github_packages(github_token, package_type="container")
except Exception as e:
    print(f"Fehler beim Laden der Konfiguration: {e}")