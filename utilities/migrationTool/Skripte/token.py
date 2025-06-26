import requests


def is_gitlab_token_valid(token, gitlab_url="https://gitlab.com"):
    """
    Überprüft, ob ein GitLab-Token gültig ist.

    :param token: Der zu überprüfende GitLab-Token.
    :param gitlab_url: Die Basis-URL der GitLab-Instanz (Standard: https://gitlab.com).
    :return: True, wenn der Token gültig ist, sonst False.
    """
    headers = {"PRIVATE-TOKEN": token}
    try:
        response = requests.get(f"{gitlab_url}/api/v4/user", headers=headers)
        return response.status_code == 200
    except requests.RequestException as e:
        print(f"Fehler bei der Anfrage: {e}")
        return False


# Beispielnutzung
gitlab_token = ""  # Ersetze dies durch den zu überprüfenden Token
gitlab_url = "https://git.rwth-aachen.de"  # URL der GitLab-Instanz

if is_gitlab_token_valid(gitlab_token, gitlab_url):
    print("Der GitLab-Token ist gültig.")
else:
    print("Der GitLab-Token ist ungültig.")