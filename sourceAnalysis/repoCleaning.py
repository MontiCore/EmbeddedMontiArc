import subprocess
import git

def run_git_filter_repo(path = "."):
    try:
        result = subprocess.run(
            ['git', 'filter-repo', '--strip-blobs-bigger-than', '100M', '--force'],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=path
        )
        print("Command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))

def split_large_files2(directory, size="90M"):
    try:
        command = f"find {directory} -type f -size +{size} -exec sh -c 'split -b {size} \"$0\" \"$0.part\"' {{}} \\;"
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Split command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))
    repo = git.Repo(directory)
    repo.git.add(all=True)
    repo.index.commit("Split large files into smaller parts")

def split_large_files(directory, size="90M"):
    try:
        command = f"find {directory} -type f -size +{size} -exec sh -c 'split -b {size} \"$0\" \"$0.part\"' {{}} \\;"
        command = f"find {directory} -path {directory}/.git -prune -o -type f -size +{size} -exec sh -c 'split -b {size} \"$0\" \"$0.part\"' {{}} \\;"
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Split command output:", result.stdout.decode())

        # Print all files that have been split
        find_command = f"find {directory} -type f -size +{size}"
        find_command = f"find {directory} -path {directory}/.git -prune -o -type f -size +{size} -print"
        find_result = subprocess.run(find_command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        files = find_result.stdout.decode().strip().split("\n")
        print("Files that have been split:")
        for file in files:
            print(f" - {file}")
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))
    repo = git.Repo(directory)
    repo.git.add(all=True)
    repo.index.commit("Split large files into smaller parts")

def concatenate_files(directory):
    try:
        command = f"find {directory} -type f -name '*.part*' | while read part; do base=$(echo \"$part\" | sed 's/.part.*//'); cat \"$part\" >> \"$base\"; rm \"$part\"; done"
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Concatenate command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error:", e.stderr.decode(errors='replace'))

if __name__ == "__main__":
    #run_git_filter_repo()
    split_large_files("../repos/MNISTCalculator")
    #concatenate_files("./test")