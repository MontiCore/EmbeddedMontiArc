import os
import shutil
import time
from unittest import TestCase

from migrationTool.gitMigration import Git
from migrationTool.migration_types import Architecture
from migrationTool.migration_types import Repo


class TestGit(TestCase):
  def tearDown(self):
    """
    Deletes the test repos created during the tests.
    """
    shutil.rmtree(os.path.join(os.getcwd(), "repos"))
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "repos")), "Test resources not deleted")

  def test_reset_remote_origin(self):
    """
    Tests replacing the value of a remote origin from a git repository.
    """
    repo = self.test_init_repo()
    self.assertFalse("origin" in repo.remotes, "No remote origin before adding")
    repo.create_remote("origin", "www.example.com")
    self.assertTrue("origin" in repo.remotes, "No remote origin after adding")
    self.assertEqual(repo.remotes.origin.url, "www.example.com", "No Remote added in test setup")
    git = Git()

    # Test
    git.reset_remote_origin(repo, "www.newexample.com")

    # Evaluate
    self.assertTrue("origin" in repo.remotes, "No remote origin after changing")
    self.assertEqual(repo.remotes.origin.url, "www.newexample.com", "New origin value not set correctly")

  def test_remove_remote_origin(self):
    """
    Tests the removal of a remote origin from a git repository.
    """
    repo = self.test_init_repo()
    self.assertFalse("origin" in repo.remotes, "No remote origin before adding")
    repo.create_remote("origin", "www.example.com")
    self.assertTrue("origin" in repo.remotes, "No remote origin after adding")
    self.assertEqual(repo.remotes.origin.url, "www.example.com", "No Remote added in test setup")
    git = Git()

    # Test
    git.remove_remote_origin(repo)

    # Evaluate
    self.assertFalse("origin" in repo.remotes, "Remote origin still exists after deletion")

  def test_init_repo(self, repo_name="test_repo"):
    """
    Tests the initialzation of a new git repository with the given name.
    :param repo_name: Name of the repository to be created
    :param delete: Whether to delete the repository after the test
    :return: If not deleted, returns the initialized repository object
    """
    git = Git()

    # Test
    repo = git.init_repo(repo_name)

    # Evaluate
    message = repo.head.commit.message
    self.assertIsNotNone(repo, "No repo object returned")
    self.assertEqual(message, "Initial commit", "New repository has correct initial commit message")
    self.assertEqual(repo.active_branch.name, "master")
    self.assertTrue(os.path.join(os.getcwd(), "repos", repo_name), "Repository directory exists")
    return repo

  def test_add_subtree(self):
    """
    Tests the addition of a single branch of a repository as a subtree to a git repository.
    """
    repoMono = self.test_init_repo(repo_name="repoMono")
    repoMono_path = os.path.join(os.getcwd(), "repos", "repoMono")

    repoA = self.test_init_repo(repo_name="repoA")
    repoA_path = os.path.join(os.getcwd(), "repos", "repoA")
    self.assertFalse(os.path.exists(os.path.join(repoA_path, "test.txt")), "File path does exist")
    with open(os.path.join(repoA_path, "test.txt"), "w") as f:
      f.write("Test content")
    self.assertTrue(os.path.exists(os.path.join(repoA_path, "test.txt")), "File path does not exist")
    repoA.git.add(all=True)
    repoA.index.commit("Test commit repoA")
    self.assertEqual(repoA.head.commit.message, "Test commit repoA", "Commit message is incorrect")

    # Test adding a subtree from repoA to repoMono
    git = Git()
    output = git.add_subtree(repoMono, "repoA", repoA_path, prefix="src")
    self.assertEqual(output, ":white_check_mark: Subtree added")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA")), "File path does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "README.md")), "File does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "test.txt")), "File does not exist")
    self.assertTrue(repoMono.head.commit.message.startswith("Add 'src/repoA/' from commit"),
                    "Commit message is incorrect")
    commits = sorted(repoMono.iter_commits(), key=lambda commit: commit.committed_date)
    head = repoMono.head.commit.hexsha
    self.assertEqual(len(commits), 4, "Monorepo has incorrect number of commits")

    # Test doing the same again, which should not change anything
    output = git.add_subtree(repoMono, "repoA", repoA_path, prefix="src")
    self.assertEqual(output, ":white_check_mark: Subtree already exists")
    self.assertEqual(repoMono.head.commit.hexsha, head, "Monorepo head commit has changed")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "README.md")), "File does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "test.txt")), "File does not exist")
    self.assertTrue(repoMono.head.commit.message.startswith("Add 'src/repoA/' from commit"),
                    "Commit message is incorrect")

    # Test adding none existing repo
    output = git.add_subtree(repoMono, "repoB", os.path.join(os.getcwd(), "repos", "repoB"), prefix="src")
    self.assertEqual(output, ":x: Subtree repository does not exist")
    self.assertEqual(repoMono.head.commit.hexsha, head, "Monorepo head commit has changed")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "README.md")), "File does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "test.txt")), "File does not exist")
    self.assertTrue(repoMono.head.commit.message.startswith("Add 'src/repoA/' from commit"),
                    "Commit message is incorrect")

  def test_add_subtree_branch(self):
    """
       Tests the addition of a branch of a repository as a subtree to a git repository. It should be saved,
       so that multiple branches can be added to the monorepo.
    """
    repoMono = self.test_init_repo(repo_name="repoMono")
    repoMono_path = os.path.join(os.getcwd(), "repos", "repoMono")

    repoA = self.test_init_repo(repo_name="repoA")
    repoA_path = os.path.join(os.getcwd(), "repos", "repoA")
    self.assertFalse(os.path.exists(os.path.join(repoA_path, "test.txt")), "File path does exist")
    with open(os.path.join(repoA_path, "test.txt"), "w") as f:
      f.write("Test content")
    self.assertTrue(os.path.exists(os.path.join(repoA_path, "test.txt")), "File path does not exist")
    repoA.git.add(all=True)
    repoA.index.commit("Test commit repoA")
    self.assertEqual(repoA.head.commit.message, "Test commit repoA", "Commit message is incorrect")
    self.assertFalse("development" in repoA.branches, "Development branch before adding")
    repoA.git.branch("development")
    self.assertTrue("development" in repoA.branches, "No development branch after adding")
    repoA.git.checkout("development")
    self.assertFalse(os.path.exists(os.path.join(repoA_path, "branch.txt")), "File path does exist")
    with open(os.path.join(repoA_path, "branch.txt"), "w") as f:
      f.write("Test content")
    repoA.git.add(all=True)
    repoA.index.commit("Add branch file")
    self.assertTrue(os.path.exists(os.path.join(repoA_path, "branch.txt")), "File path does not exist")

    # Test adding a subtree from repoA to repoMono
    git = Git()
    output = git.add_subtree_branch(repoMono, "repoA", repoA_path, branch="master", prefix="src")
    self.assertEqual(output, ":white_check_mark: Subtree added")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "master")), "File path does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "master", "README.md")),
                    "File does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "master", "test.txt")),
                    "File does not exist")
    self.assertTrue(repoMono.head.commit.message.startswith("Add 'src/repoA/master/' from commit"),
                    "Commit message is incorrect")
    commits = sorted(repoMono.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 4, "Monorepo has incorrect number of commits")

    # Test adding second branch
    output = git.add_subtree_branch(repoMono, "repoA", repoA_path, branch="development", prefix="src")
    self.assertEqual(output, ":white_check_mark: Subtree added")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "development")),
                    "File path does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "development", "README.md")),
                    "File does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "development", "test.txt")),
                    "File does not exist")
    self.assertTrue(os.path.exists(os.path.join(repoMono_path, "src", "repoA", "development", "branch.txt")),
                    "File does not exist")
    self.assertTrue(repoMono.head.commit.message.startswith("Add 'src/repoA/development/' from commit"),
                    "Commit message is incorrect")
    commits = sorted(repoMono.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 6, "Monorepo has incorrect number of commits")
    head = repoMono.head.commit.hexsha

    # Test adding the same branch again
    output = git.add_subtree_branch(repoMono, "repoA", repoA_path, branch="development", prefix="src")
    self.assertEqual(output, ":white_check_mark: Subtree already exists")
    self.assertEqual(repoMono.head.commit.hexsha, head, "Monorepo head commit has changed")

    # Test adding none existent repo
    output = git.add_subtree_branch(repoMono, "repoB", os.path.join(os.getcwd(), "repos", "repoB"), branch="master",
                                    prefix="src")
    self.assertEqual(output, ":x: Subtree repository does not exist")
    self.assertEqual(repoMono.head.commit.hexsha, head, "Monorepo head commit has changed")

  def test_add_repos_as_subtree(self):
    """
      Test the addition of multiple repositories as subtrees to a monorepo.
    """

    # Setup repos to be added as subtrees: RepoA 2 Branches, RepoB 1 Branch
    repoA = self.test_init_repo(repo_name="repoA")
    repoA_path = os.path.join(os.getcwd(), "repos", "repoA")
    self.assertFalse(os.path.exists(os.path.join(repoA_path, "test.txt")), "File path does exist")
    with open(os.path.join(repoA_path, "test.txt"), "w") as f:
      f.write("Test content")
    self.assertTrue(os.path.exists(os.path.join(repoA_path, "test.txt")), "File path does not exist")
    repoA.git.add(all=True)
    repoA.index.commit("Test commit repoA")
    self.assertEqual(repoA.head.commit.message, "Test commit repoA", "Commit message is incorrect")
    self.assertFalse("development" in repoA.branches, "Development branch before adding")
    repoA.git.branch("development")
    self.assertTrue("development" in repoA.branches, "No development branch after adding")
    repoA.git.checkout("development")
    self.assertFalse(os.path.exists(os.path.join(repoA_path, "branch.txt")), "File path does exist")
    with open(os.path.join(repoA_path, "branch.txt"), "w") as f:
      f.write("Test content")
    repoA.git.add(all=True)
    repoA.index.commit("Add branch file")
    self.assertTrue(os.path.exists(os.path.join(repoA_path, "branch.txt")), "File path does not exist")

    repoB = self.test_init_repo(repo_name="repoB")
    repoB_path = os.path.join(os.getcwd(), "repos", "repoB")
    self.assertFalse(os.path.exists(os.path.join(repoB_path, "test.txt")), "File path does exist")
    with open(os.path.join(repoB_path, "test.txt"), "w") as f:
      f.write("Test content")
    self.assertTrue(os.path.exists(os.path.join(repoB_path, "test.txt")), "File path does not exist")
    repoB.git.add(all=True)
    repoB.index.commit("Test commit repoB")
    self.assertEqual(repoB.head.commit.message, "Test commit repoB", "Commit message is incorrect")

    # Create mockup Architecture object
    architecture = Architecture("mock_architecture")
    self.assertEqual(architecture.repos, {}, "Architecture repos should be empty at start")

    repositoryA = Repo("repoA", "1", [""], repoA_path, "test", ["master", "development"], [], [])
    architecture.add_repo(repositoryA)
    repositoryB = Repo("repoB", "2", [""], repoB_path, "test/abc", ["master"], [], [])
    architecture.add_repo(repositoryB)

    # Test
    git = Git()
    git.add_repos_as_subtree("repoMono", "test", architecture, ["1", "2"])

    # Compare resulting repository structure
    repo = git.init_repo("repoMono")
    migrationBranch = None
    for branch in repo.heads:
      if "Migration" in branch.name:
        migrationBranch = branch.name
        break
    self.assertIsNotNone(migrationBranch)
    repo.git.checkout(migrationBranch)
    commits = sorted(repo.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 9, "Wrong number of commits in monorepo")
    self.assertTrue(os.path.exists(os.path.join(os.getcwd(), "repos", "repoMono", "repoA", "master")),
                    "RepoA branch master does not exist")
    self.assertTrue(os.path.exists(os.path.join(os.getcwd(), "repos", "repoMono", "repoA", "development")),
                    "RepoA branch development does not exist")
    self.assertTrue(os.path.exists(os.path.join(os.getcwd(), "repos", "repoMono", "abc", "repoB")),
                    "RepB does not exist")
