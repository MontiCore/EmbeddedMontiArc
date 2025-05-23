import os
import sys
import subprocess
import requests

# 🔹 CHANGE THIS: Set your private GitLab server URL
GITLAB_SERVER = "https://git.rwth-aachen.de"

# 🔹 CHANGE THIS: Set your GitLab Personal Access Token (or use environment variable)
GITLAB_TOKEN = "glpat-1E9aKZussdM6-vaqwDGk"


def extract_project_info(repo_url):
    """Extracts project path from the GitLab repo URL."""
    if GITLAB_SERVER not in repo_url:
        print(f"❌ Error: Repo URL must be from {GITLAB_SERVER}")
        sys.exit(1)

    project_path = repo_url.replace(GITLAB_SERVER + "/", "").rstrip(".git")
    return project_path


def clone_gitlab_repo(repo_url, destination="cloned_repo"):
    """Clones the private GitLab repository."""
    if os.path.exists(destination):
        print(f"⚠️  Directory '{destination}' already exists. Skipping clone.")
        return destination

    print(f"🔄 Cloning GitLab repository: {repo_url}...")
    try:
        subprocess.run(["git", "clone", repo_url, destination], check=True)
        print("✅ Repository cloned successfully!")
        return destination
    except subprocess.CalledProcessError:
        print("❌ Error: Failed to clone repository.")
        sys.exit(1)


def get_gitlab_project_id(project_path):
    """Fetches the GitLab project ID using the API."""
    url = f"{GITLAB_SERVER}/api/v4/projects/{project_path.replace('/', '%2F')}"
    headers = {"PRIVATE-TOKEN": GITLAB_TOKEN}

    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        return response.json().get("id")
    else:
        print(f"❌ Error: Failed to fetch project ID ({response.status_code})")
        sys.exit(1)


def list_maven_packages(project_id):
    """Lists published Maven packages in the GitLab package registry."""
    print("\n🔍 Fetching published Maven packages...")
    url = f"{GITLAB_SERVER}/api/v4/projects/{project_id}/packages"
    headers = {"PRIVATE-TOKEN": GITLAB_TOKEN}

    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        packages = response.json()
        if packages:
            print("✅ Found Maven Packages:")
            for pkg in packages:
                print(f"   - {pkg['name']} (Version: {pkg['version']})")
        else:
            print("❌ No Maven packages found.")
    else:
        print(f"❌ Error: Failed to fetch Maven packages ({response.status_code})")


def list_docker_images(project_id):
    """Lists published Docker images in the GitLab container registry."""
    print("\n🔍 Fetching published Docker images...")
    url = f"{GITLAB_SERVER}/api/v4/projects/{project_id}/registry/repositories"
    headers = {"PRIVATE-TOKEN": GITLAB_TOKEN}

    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        repositories = response.json()
        if repositories:
            print("✅ Found Docker Images:")
            for repo in repositories:
                repo_id = repo["id"]
                tags_url = f"{GITLAB_SERVER}/api/v4/projects/{project_id}/registry/repositories/{repo_id}/tags"
                tags_response = requests.get(tags_url, headers=headers)

                if tags_response.status_code == 200:
                    tags = tags_response.json()
                    for tag in tags:
                        print(f"   - {repo['path']}:{tag['name']}")
                else:
                    print(f"   - {repo['path']} (No tags found)")
        else:
            print("❌ No Docker images found.")
    else:
        print(f"❌ Error: Failed to fetch Docker images ({response.status_code})")


if __name__ == "__main__":


    gitlab_repo_url = "https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/mnistcalculator"
    project_path = extract_project_info(gitlab_repo_url)

    print(f"📂 GitLab Project: {project_path}")

    # Clone repo (optional)
    #clone_gitlab_repo(gitlab_repo_url)

    # Get GitLab project ID
    project_id = get_gitlab_project_id(project_path)

    # List published Maven packages & Docker images
    list_maven_packages(project_id)
    list_docker_images(project_id)
