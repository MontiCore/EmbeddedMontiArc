import requests
from migrationTool.migration_types.Config import Config

# Replace these with your actual values
config = Config('../config.yaml')

GITHUB_TOKEN = config.targetToken
OWNER = config.targetUser
REPO = config.monorepoName

headers = {'Authorization': f'token {GITHUB_TOKEN}', 'Accept': 'application/vnd.github+json'}


# Get all workflow runs
def get_workflow_runs():
  url = f'https://api.github.com/repos/{OWNER}/{REPO}/actions/runs'
  response = requests.get(url, headers=headers)
  response.raise_for_status()
  return response.json().get('workflow_runs', [])


# Delete a workflow run by ID
def delete_workflow_run(run_id):
  url = f'https://api.github.com/repos/{OWNER}/{REPO}/actions/runs/{run_id}'
  response = requests.delete(url, headers=headers)
  if response.status_code == 204:
    print(f'✅ Successfully deleted run {run_id}')
  else:
    print(f'❌ Failed to delete run {run_id}: {response.status_code} {response.text}')


def main():
  runs = get_workflow_runs()
  print(f'Found {len(runs)} workflow runs.')

  for run in runs:
    delete_workflow_run(run['id'])


if __name__ == '__main__':
  main()
