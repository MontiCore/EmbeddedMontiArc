import os
import subprocess
import sys

from tqdm import tqdm

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
    print("\nScanning for large files in the current working directory...")
    try:
        result = subprocess.check_output(['git', '-C', repo_path, 'ls-files', '-s']).decode()
        for line in result.splitlines():
            parts = line.split()
            if len(parts) >= 4:
                file_path = os.path.join(repo_path, parts[3])
                if os.path.exists(file_path):
                    size = os.path.getsize(file_path)
                    if size > SIZE_LIMIT:
                        print(f"Large file found: {file_path} ({size / (1024 * 1024):.2f} MB)")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")


def findLargeFilesInHistory(repo_path):
    """Find large files in the Git commit history."""
    try:
        # Get all blob objects in the repo history
        blobs = subprocess.check_output(['git', '-C', repo_path, 'rev-list', '--objects', '--all']).decode()
        blob_lines = blobs.splitlines()

        output = ""
        for line in tqdm(blob_lines, desc="Scanning history"):
            parts = line.split()
            if len(parts) == 2:
                blob_hash, file_path = parts
                # Get the file size
                size_output = subprocess.check_output(
                    ['git', '-C', repo_path, 'cat-file', '-s', blob_hash]).decode().strip()
                size = int(size_output)
                if size > SIZE_LIMIT:
                    output+=(f"   -  {file_path} ({size / (1024 * 1024):.2f} MB)\n")

        if output:
            print("Large files found in history:")
            print(output)
        else:
            print("No large files found in history.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    repo_path = '../repos/MNISTCalculator'

    # Validate the repository
    repo_root = get_git_root(repo_path)
    if repo_root:
        print(f"Git Repository: {repo_root}")
        find_large_files_in_repo(repo_root)
        findLargeFilesInHistory(repo_root)
    print("✅ Scan completed.")