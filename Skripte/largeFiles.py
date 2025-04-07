import os
import subprocess
import sys

# Define size limit (100MB)
SIZE_LIMIT = 100 * 1024 * 1024  # 100MB in bytes


def get_git_root(repo_path):
    """Check if the provided path is a Git repository and return its root."""
    try:
        root = subprocess.check_output(['git', '-C', repo_path, 'rev-parse', '--show-toplevel']).strip().decode()
        return root
    except subprocess.CalledProcessError:
        print(f"❌ Error: '{repo_path}' is not a valid Git repository.")
        return None


def find_large_files_in_repo(repo_path):
    """Find large files in the working directory (latest commit)."""
    print("\n🔍 Scanning for large files in the current working directory...")
    try:
        result = subprocess.check_output(['git', '-C', repo_path, 'ls-files', '-s']).decode()
        for line in result.splitlines():
            parts = line.split()
            if len(parts) >= 4:
                file_path = os.path.join(repo_path, parts[3])
                if os.path.exists(file_path):
                    size = os.path.getsize(file_path)
                    if size > SIZE_LIMIT:
                        print(f"⚠️  Large file found: {file_path} ({size / (1024 * 1024):.2f} MB)")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")


def find_large_files_in_history(repo_path):
    """Find large files in the Git commit history."""
    print("\n🔍 Scanning for large files in commit history... (This may take time)")
    try:
        # Get all blob objects in the repo history
        blobs = subprocess.check_output(['git', '-C', repo_path, 'rev-list', '--objects', '--all']).decode()

        for line in blobs.splitlines():
            parts = line.split()
            if len(parts) == 2:
                blob_hash, file_path = parts
                # Get the file size
                size_output = subprocess.check_output(
                    ['git', '-C', repo_path, 'cat-file', '-s', blob_hash]).decode().strip()
                size = int(size_output)
                if size > SIZE_LIMIT:
                    print(f"⚠️  Large file in history: {file_path} ({size / (1024 * 1024):.2f} MB)")

    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    repo_path = '/mnt/c/Users/blumd/Desktop/Public migration/mnistcalculator'

    # Validate the repository
    repo_root = get_git_root(repo_path)
    if repo_root:
        print(f"📂 Git Repository: {repo_root}")
        find_large_files_in_repo(repo_root)
        find_large_files_in_history(repo_root)
