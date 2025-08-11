import os
import shutil
from unittest import TestCase

from migrationTool.git_migration import Git
from migrationTool.git_migration.largeFiles import run_git_filter_repo


class Test(TestCase):
  def tearDown(self):
    """
    Deletes the test repos created during the tests.
    """
    shutil.rmtree(os.path.join(os.getcwd(), "repos"))
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "repos")), "Test resources not deleted")

  def test_run_git_filter_repo(self):
    # Create repo with 3 new commits introducing large files
    git = Git()
    repo = git.init_repo("largeRepo")
    repo_path = os.path.join(os.getcwd(), "repos", "largeRepo")
    self.assertIsNotNone(repo, "No repo object returned")
    self.assertTrue(os.path.exists(repo_path), "Repo in filesystem created")

    self.create_sparse_file(os.path.join(repo_path, "a.txt"), 99)
    repo.git.add(all=True)
    repo.index.commit("90mb file")
    self.create_sparse_file(os.path.join(repo_path, "b.txt"), 101)
    repo.git.add(all=True)
    repo.index.commit("110mb file")
    self.create_sparse_file(os.path.join(repo_path, "c.txt"), 10)
    self.create_sparse_file(os.path.join(repo_path, "d.txt"), 200)
    repo.git.add(all=True)
    repo.index.commit("Two files")

    self.assertTrue(os.path.exists(os.path.join(repo_path, "a.txt")))
    self.assertTrue(os.path.exists(os.path.join(repo_path, "b.txt")))
    self.assertTrue(os.path.exists(os.path.join(repo_path, "c.txt")))
    self.assertTrue(os.path.exists(os.path.join(repo_path, "d.txt")))

    # Run method
    run_git_filter_repo(repo_path)

    self.assertTrue(os.path.exists(os.path.join(repo_path, "a.txt")))
    self.assertFalse(os.path.exists(os.path.join(repo_path, "b.txt")))
    self.assertTrue(os.path.exists(os.path.join(repo_path, "c.txt")))
    self.assertFalse(os.path.exists(os.path.join(repo_path, "d.txt")))

    commits = sorted(repo.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 3, "Monorepo has incorrect number of commits")

  def create_sparse_file(self, filename: str, size_in_mb: int):
    size_in_bytes = size_in_mb * 1024 * 1024
    with open(filename, "wb") as f:
      f.seek(size_in_bytes - 1)
      f.write(b'\0')
    print(f"{filename} created with size {size_in_mb} MB ({size_in_bytes} bytes).")
