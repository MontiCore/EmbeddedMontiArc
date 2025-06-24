import shutil
from unittest import TestCase

from migrationTool.gitMigration.Git import Git
from migrationTool.gitMigration.repoCleaning import *


class Test(TestCase):
  def setUp(self):
    git = Git()
    self.repo = git.init_repo("testRepo")
    self.repo_path = os.path.join(os.getcwd(), "repos", "testRepo")
    self.assertIsNotNone(self.repo)
    self.assertTrue(os.path.exists(self.repo_path))

  def tearDown(self):
    shutil.rmtree(os.path.join(os.getcwd(), "repos"))
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "repos")))

  def test_remove_lfs(self):
    """
    Tests the removal of git lfs from a repository
    """
    self.repo.git.lfs("install")
    self.repo.git.lfs("track", "*.txt")
    self.create_sparse_file(os.path.join(self.repo_path, "large.txt"), 130)
    self.repo.git.add(all=True)
    self.repo.index.commit("Added large file")
    self.assertTrue(os.path.exists(os.path.join(self.repo_path, "large.txt")), "Large file does not exist")
    commits = sorted(self.repo.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 2, "False number of commits")
    lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=self.repo_path, capture_output=True, text=True)
    self.assertIsNotNone(lfs_check.stdout, "LFS not correctly installed")

    # Test
    remove_lfs(self.repo_path)
    remove_lfs_from_gitattributes(self.repo_path)

    # Evaluate
    lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=self.repo_path, capture_output=True, text=True)
    self.assertIsNotNone(lfs_check.stdout, "LFS not correctly removed")
    with open(os.path.join(self.repo_path, ".gitattributes"), "r") as f:
      lines = f.readlines()
    self.assertNotIn("filter=lfs", lines, "LFS not correctly removed from gitattributes")
    commits = sorted(self.repo.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 3, "False number of commits")

  def test_split_large_files(self):
    """
    Tests splitting large files into smaller parts
    """
    self.create_sparse_file(os.path.join(self.repo_path, "large.txt"), 130)
    self.repo.git.add(all=True)
    self.repo.index.commit("Added large file")
    self.assertTrue(os.path.exists(os.path.join(self.repo_path, "large.txt")), "Large file does not exist")
    commits = sorted(self.repo.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 2, "False number of commits")

    # Test
    split_large_files(self.repo_path)

    # Evaluate
    commits = sorted(self.repo.iter_commits(), key=lambda commit: commit.committed_date)
    self.assertEqual(len(commits), 3, "False number of commits")
    self.assertTrue(os.path.exists(os.path.join(self.repo_path, "large.txt.parta")), ".parta does not exist")
    self.assertTrue(os.path.exists(os.path.join(self.repo_path, "large.txt.partb")), ".partb does not exist")
    self.assertFalse(os.path.exists(os.path.join(self.repo_path, "large.txt.partc")), ".partc does falsely exist")

  def create_sparse_file(self, filename: str, size_in_mb: int):
    size_in_bytes = size_in_mb * 1024 * 1024
    with open(filename, "wb") as f:
      f.seek(size_in_bytes - 1)
      f.write(b'\0')
    print(f"{filename} created with size {size_in_mb} MB ({size_in_bytes} bytes).")
