import requests
from migrationTool.migration_types.Config import Config

# Replace these with your actual values
config = Config('../config.yaml')

GITHUB_TOKEN = config.targetToken
OWNER = config.targetUser
REPO = config.monorepoName

headers = {'Authorization': f'token {GITHUB_TOKEN}', 'Accept': 'application/vnd.github+json'}


# Get all workflow runs with pagination
def get_workflow_runs():
    workflow_runs = []
    page = 1
    while True:
        url = f'https://api.github.com/repos/{OWNER}/{REPO}/actions/runs?per_page=100&page={page}'
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        runs = response.json().get('workflow_runs', [])
        if not runs:
            break
        workflow_runs.extend(runs)
        page += 1
    return workflow_runs


# Cancel a workflow run by ID
def cancel_workflow_run(run_id):
    url = f'https://api.github.com/repos/{OWNER}/{REPO}/actions/runs/{run_id}/cancel'
    response = requests.post(url, headers=headers)
    if response.status_code == 202:
        print(f'✅ Successfully canceled run {run_id}')
    else:
        print(f'❌ Failed to cancel run {run_id}: {response.status_code} {response.text}')


def main():
    runs = get_workflow_runs()
    print(f'Found {len(runs)} workflow runs.')

    for run in runs:
        if run['status'] == 'queued':
            print(f'Canceling queued run {run["id"]}...')
            cancel_workflow_run(run['id'])


if __name__ == '__main__':
    main()