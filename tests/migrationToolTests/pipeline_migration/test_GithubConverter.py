import os
import shutil
from unittest import TestCase

from migrationTool.migration_types import Architecture
from migrationTool.pipeline_migration.GitlabCIImporter import GitlabCIImporter
from migrationTool.pipeline_migration.GithubConverter import GithubActionConverter

"""
DO NOT REFORMAT THIS FILE! OTHERWISE THE TESTS WILL FAIL!
"""


class TestGithubActionConverter(TestCase):
  def setUp(self):
    current_file_dir = os.path.dirname(__file__)
    testresources_path = os.path.abspath(os.path.join(current_file_dir, "../../testRessources/GitHubConverter"))
    shutil.copytree(testresources_path, os.path.join(os.getcwd(), "TEST"))
    self.architecture = Architecture.load_architecture(os.path.join(os.getcwd(), "TEST", "architecture.yaml"))
    with open(os.path.join(os.getcwd(), "TEST", ".gitlab-ci.yml")) as file:
      importer = GitlabCIImporter()
      self.pipeline = importer.get_pipeline(file)
    compatibleImages = {"ubuntu:latest", "docker:latest"}
    self.github_converter = GithubActionConverter(self.architecture, self.pipeline, compatibleImages)

  def tearDown(self):
    shutil.rmtree(os.path.join(os.getcwd(), "TEST"))

  def test_parse_pipeline(self):
    """
    Tests that the pipeline is parsed correctly. It should contain the correct name, triggers, variables, and jobs.
    """
    pipeline = self.github_converter.parse_pipeline("1")
    self.assertIsNotNone(pipeline)
    self.assertIsInstance(pipeline, str)

    # Test that it contains the correct name
    self.assertIn("name: RepoA\n", pipeline)
    # Test that it declares triggers
    self.assertIn("on:\n", pipeline)
    # Test that it contains the correct variables
    self.assertIn(
      "env:\n  CI_PROJECT_ID : 1\n  CI_API_V4_URL : https://git.rwth-aachen.de/api/v4\n  GITLABTOKEN : ${{ "
      "secrets.GITLABTOKEN }}\n  TEST : test",
      pipeline)
    # Test that it contains the correct jobs
    self.assertIn("FileChanges", pipeline)
    self.assertIn("test_phase", pipeline)
    self.assertIn("build_phase", pipeline)
    self.assertNotIn("deploy_phase", pipeline)
    self.assertIn("variable_job", pipeline)
    self.assertIn("artifact_job", pipeline)
    self.assertIn("artifact_advanced_job", pipeline)
    self.assertIn("image_native_job", pipeline)
    self.assertIn("image_manual_migrated_job", pipeline)
    self.assertIn("image_manual_not_migrated_job", pipeline)
    self.assertIn("only_branch_job", pipeline)
    self.assertIn("except_branch_job", pipeline)
    self.assertIn("only_files_job", pipeline)
    self.assertIn("except_files_job", pipeline)
    self.assertIn("only_except_files_job", pipeline)
    self.assertIn("need_job", pipeline)
    self.assertIn("pages", pipeline)
    self.assertIn("trigger_job", pipeline)
    self.assertIn("docker_not_migrated_job", pipeline)
    self.assertIn("docker_migrated_job", pipeline)
    self.assertIn("maven_job", pipeline)
    self.assertIn("report_job", pipeline)
    self.assertIn("rules_job", pipeline)
    self.assertIn("allow_failure_job", pipeline)
    self.assertIn("dependencies_job", pipeline)
    # Test that FileChange job provides correct outputs
    output_definition = """outputs:
      runonly_files_job: ${{steps.only_files_job.outputs.run}}
      runexcept_files_job: ${{steps.except_files_job.outputs.run}}
      runonly_except_files_job: ${{steps.only_except_files_job.outputs.run}}
      runrules_job: ${{steps.rules_job.outputs.run}}"""
    self.assertIn(output_definition, pipeline)

  def test_parse_variable_job(self):
    """
    Tests that the variable job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["variable_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  variable_job:
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Running tests..."
            echo "$TEST"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_artifact_job(self):
    """
    Tests that the artifact job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["artifact_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  artifact_job:
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Building artifacts..."
            echo "Artifact content" > artifact.txt
      - name: Upload artifacts
        uses: actions/upload-artifact@v4
        if: success()
        with:
          name: artifact_job
          retention-days: 7
          path: |
            artifact.txt
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_artifact_advanced_job(self):
    """
    Tests that the artifact job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["artifact_advanced_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  artifact_advanced_job:
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Building artifacts..."
            echo "Artifact content" > artifact.txt
      - name: Upload artifacts
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: artifact_advanced_job
          retention-days: 10 
          path: |
            artifact.txt
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_image_native_job(self):
    """
    Tests that the image native job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["image_native_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  image_native_job:
    runs-on: ubuntu-latest
    container:
      image: docker:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Deploying image..."
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_image_manual_migrated_job(self):
    """
    Tests that the image manual migrated job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["image_manual_migrated_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  image_manual_migrated_job:
    runs-on: ubuntu-latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Start Docker Container
        run: |
          echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin
          docker pull ghcr.io/davidblm/repoa/embedded_montiarc:latest
          docker run --name build-container -d -v $(pwd):/workspace --network=host  -e CI_PROJECT_ID=$CI_PROJECT_ID -e CI_API_V4_URL=$CI_API_V4_URL -e GITLABTOKEN=${{ secrets.GITLABTOKEN }} ghcr.io/davidblm/repoa/embedded_montiarc:latest tail -f /dev/null
      - name: Script
        env:
          SCRIPT: |
            cd /workspace
            echo "Deploying private image..."
        run: docker exec build-container bash -c "set -e; $SCRIPT"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_image_manual_not_migrated_job(self):
    """
    Tests that the image manual not migrated job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["image_manual_not_migrated_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  image_manual_not_migrated_job:
    runs-on: ubuntu-latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Start Docker Container
        run: |
          docker pull registry.gitlab.com/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc2:latest
          docker run --name build-container -d -v $(pwd):/workspace --network=host  -e CI_PROJECT_ID=$CI_PROJECT_ID -e CI_API_V4_URL=$CI_API_V4_URL -e GITLABTOKEN=${{ secrets.GITLABTOKEN }} registry.gitlab.com/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc2:latest tail -f /dev/null
      - name: Script
        env:
          SCRIPT: |
            cd /workspace
            echo "Deploying private image..."
        run: docker exec build-container bash -c "set -e; $SCRIPT"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_only_branch_job(self):
    """
    Tests that the only branch job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["only_branch_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  only_branch_job:
    if: ${{ !cancelled() && github.ref_name == 'main' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job runs only on the 'main' branch"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_except_branch_job(self):
    """
    Tests that the except branch job is correctly parsed
        """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["except_branch_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  except_branch_job:
    if: ${{ github.ref_name != 'main' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job runs on all branches except 'main'"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_only_files_job(self):
    """
    Tests that the only files job is correctly parsed
    """
    self.github_converter.file_change_job_needed = True

    pipeline = self.github_converter.parse_job(self.pipeline.jobs["only_files_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  only_files_job:
    needs: FileChanges
    if: ${{ !cancelled() && needs.FileChanges.outputs.runonly_files_job == 'true' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job runs only if files in the 'src/' directory are changed"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_except_files_job(self):
    """
    Tests that the except files job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["except_files_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  except_files_job:
    if: ${{ !cancelled() && needs.FileChanges.outputs.runexcept_files_job == 'true' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job runs on all changes except files in the 'docs/' directory"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_only_except_files_job(self):
    self.github_converter.file_change_job_needed = True
    """
    Tests that the only except files job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["only_except_files_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  only_except_files_job:
    needs: FileChanges
    if: ${{ !cancelled() && needs.FileChanges.outputs.runonly_except_files_job == 'true' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job runs only if files in the 'src/' directory are changed, but not if files in the 'docs/' directory are changed"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_need_job(self):
    """
    Tests that the need job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["need_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  need_job:
    needs: artifact_job
    if: ${{ !cancelled() && needs.artifact_job.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Download artifacts
        uses: actions/download-artifact@v4
        with:
          name: artifact_job
          path: |
            artifact.txt
      - name: Script
        shell: bash
        run: |
            echo "This job needs the 'artifact_job' to complete successfully before it runs and should download it"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_pages_job(self):
    """
    Tests that the pages job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["pages"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  pages:
    needs: build_phase
    if: ${{ !cancelled() && needs.build_phase.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    permissions:
      pages: write
      id-token: write
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Deploying pages..."
            mkdir public
            echo "This is a test page" > public/index.html
      - name: Upload Pages
        uses: actions/upload-pages-artifact@v3
        with:
          path: public/
      - name: Deploy to Pages
        uses: actions/deploy-pages@v4
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_trigger_job(self):
    """
    Tests that the trigger job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["trigger_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)

    print(pipeline)
    # @formatter:off
    expected = """  trigger_job:
    needs: build_phase
    if: ${{ !cancelled() && needs.build_phase.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Triggering another repository..."
      - name: Record trigger time
        id: trigger_time
        run: echo "time=$(date -u +'%Y-%m-%dT%H:%M:%SZ')" >> $GITHUB_OUTPUT
      - name: Trigger another-repo pipeline
        run: gh workflow run $WORKFLOW_FILE --repo $REPO  --ref $BRANCH
        env:
          WORKFLOW_FILE: another-repo.yml
          BRANCH: main
          REPO: another-repo
          GH_TOKEN: ${{github.token}}
      - name: Wait for workflow to appear
        run: sleep 20
      - name: Get latest run ID of Child Workflow after trigger time
        id: get_run
        run: |
          RUN_ID=$(gh run list --workflow=$WORKFLOW_FILE --branch=$BRANCH --repo=$REPO --json databaseId,createdAt \\
          --jq '[.[] | select(.createdAt > "${{ steps.trigger_time.outputs.time }}")] | sort_by(.createdAt) | last.databaseId')
          echo "Run ID: $RUN_ID"
          echo "run_id=$RUN_ID" >> $GITHUB_OUTPUT
        env:
          GH_TOKEN: ${{github.token}}
          WORKFLOW_FILE: another-repo.yml
          BRANCH: main
          REPO: another-repo
      - name: Wait for Child Workflow to finish
        run: |
          while true; do
            STATUS=$(gh run view "${{ steps.get_run.outputs.run_id }}" --repo=$REPO --json status --jq '.status')
            echo "Current status: $STATUS"
            if [[ "$STATUS" == "completed" ]]; then
              break
            fi
            sleep 60
          done
        env:
          GH_TOKEN: ${{github.token}}
          REPO: another-repo
      - name: Check Child Workflow result
        run: |
          RESULT=$(gh run view "${{ steps.get_run.outputs.run_id }}" --repo=$REPO --json conclusion --jq '.conclusion')
          echo "Child Workflow result: $RESULT"
          if [[ "$RESULT" != "success" ]]; then
            echo "Child Workflow failed"
            exit 1
          fi
        env:
          GH_TOKEN: ${{github.token}}
          REPO: another-repo
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_docker_not_migrated_job(self):
    """
    Tests that the docker not migrated job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["docker_not_migrated_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  docker_not_migrated_job:
    needs: test_phase
    if: ${{ !cancelled() && needs.test_phase.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Building Docker image..."
            docker login registry.git.rwth-aachen.de -u someUserName -p abc
            docker build -t registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/mxnet170-dgl:v0.0.1 -f Dockerfile-dgl .
            docker push registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/mxnet170-dgl:v0.0.1
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_docker_migrated_job(self):
    """
    Tests that the docker migrated job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["docker_migrated_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  docker_migrated_job:
    needs: test_phase
    if: ${{ !cancelled() && needs.test_phase.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Building Docker image..."
            echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin
            LOWERCASE_OWNER=$(echo "${{ github.repository_owner }}" | tr "[:upper:]" "[:lower:]")
            docker build -t ghcr.io/$LOWERCASE_OWNER/repoa/embedded_montiarc:latest -f Dockerfile .
            docker push ghcr.io/$LOWERCASE_OWNER/repoa/embedded_montiarc:latest
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_maven_job(self):
    """
    Tests that the maven job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["maven_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  maven_job:
    needs: test_phase
    if: ${{ !cancelled() && needs.test_phase.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Building Maven project..."
            mvn clean install -s settings.xml
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_report_job(self):
    """
    Tests that the report job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["report_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    expected = """  report_job:
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "Generating test report..."
            mkdir reports
            echo "Test report content" > reports/test-report.txt
      - name: Reporting
        uses: dorny/test-reporter@v2
        continue-on-error: true
        if : ${{ always() }}
        with:
          name: report_job
          path: |
            reports/test-report.xml
          reporter: java-junit
"""
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_rules_job(self):
    """
    Tests that the rules job is correctly parsed
    """
    self.github_converter.file_change_job_needed = True
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["rules_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  rules_job:
    needs: FileChanges
    if: ${{needs.FileChanges.outputs.runrules_job == 'true' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job runs based on rules"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_allow_failure_job(self):
    # Tests that the allow failure job is correctly parsed
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["allow_failure_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  allow_failure_job:
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    continue-on-error: true
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        continue-on-error: true
        shell: bash
        run: |
            echo "This job allows failure"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)

  def test_parse_dependencies_job(self):
    """
    Tests that the dependencies job is correctly parsed
    """
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["dependencies_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    # @formatter:off
    expected = """  dependencies_job:
    needs: need_job
    if: ${{ !cancelled() && needs.need_job.result == 'success' }}
    runs-on: ubuntu-latest
    container:
      image: ubuntu:latest
    timeout-minutes: 120
    steps:
      - name: Checkout latest commit
        uses: actions/checkout@v4
        with:
          fetch-depth: 1
      - name: Script
        shell: bash
        run: |
            echo "This job depends on the 'artifact_job' and should download its artifacts"
"""
    # @formatter:on
    self.assertMultiLineEqual(pipeline, expected)
