import re

# Define the pattern to search for environment variables
env_var_pattern = re.compile(r'\$([A-Z_]+)')

def find_env_vars_in_pipeline(file_path):
    env_vars_found = set()

    with open(file_path, 'r', errors='ignore') as file:
        content = file.read()
        env_vars = env_var_pattern.findall(content)
        env_vars_found.update(env_vars)

    # Filter out local variables
    env_vars_filtered = {var for var in env_vars_found if var.isupper()}

    return env_vars_filtered

if __name__ == "__main__":
    for file_path in ['../repos/EMADL2CPP/.gitlab-ci.yml', '../repos/MNISTCalculator/.gitlab-ci.yml']:

        #file_path = '../.gitlab-ci.yml'  # Path to the GitLab CI configuration file
        env_vars = find_env_vars_in_pipeline(file_path)

        print("Environment variables found:")
        for env_var in env_vars:
            print(f"  {env_var}")