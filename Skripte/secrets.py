import os
import re

# Define the pattern to search for environment variables in Maven files
env_var_pattern = re.compile(r'\$\{env\.(\w+)\}')

def find_env_vars_in_file(file_path):
    env_vars = set()
    with open(file_path, 'r', errors='ignore') as file:
        content = file.read()
        matches = env_var_pattern.findall(content)
        env_vars.update(matches)
    return env_vars

def find_env_vars_in_repo(repo_path):
    env_vars_found = {}
    for root, _, files in os.walk(repo_path):
        for file in files:
            if file.endswith('.xml'):
                file_path = os.path.join(root, file)
                env_vars = find_env_vars_in_file(file_path)
                if env_vars:
                    env_vars_found[file_path] = env_vars
    return env_vars_found

if __name__ == "__main__":
    repo_path = '../repos/EMADL2CPP'  # Path to the git repository
    print(repo_path)
    env_vars = find_env_vars_in_repo(repo_path)
    env_All = set()
    for file_path, env_vars in env_vars.items():
        print(f"Environment variables found in {file_path}:")
        for env_var in env_vars:
            print(f"  {env_var}")
        env_All.update(env_vars)
    print("--------------------------")
    print(env_All)