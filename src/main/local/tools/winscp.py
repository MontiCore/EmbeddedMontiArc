import subprocess
from pathlib import Path
import time
from typing import List
from config import config

def current_path():
    return Path( __file__ ).parent.resolve()

def upload_files(files: List[str], local_path: str, remote_path: str, delete_sources = False):
    """ Uploads a list of files to the cluster into the specified path

        Parameters:
        files (List[str]): the list of files to copy to the cluster
        remote_path (str): the directory where to copy the files to
    """
    if isinstance(files, str):
        files = [files]

    logfolder = Path(config.get("DEFAULT", "LogFolder")).joinpath("transferP2C.log")
    script = current_path().joinpath("winscp/PC_to_Cluster.txt")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    putty_ssh_key = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.ppk")
    putty_ssh_key_passphrase = config.get("DEFAULT", "ClusterKeyPassword")

    if delete_sources:
        files.insert(0,"-delete")

    filestring = " ".join(files)

    print("Copying files to cluster...")
    return subprocess.run(
        [
            'winscp.com', '/ini=nul', f'/log={logfolder}', f'/script={script}', '/parameter',
            cluster_user,
            putty_ssh_key,
            putty_ssh_key_passphrase,
            local_path,
            remote_path,
            filestring
        ] 
    )

def download_files(files: List[str], local_path: str, remote_path: str, delete_sources=False):
    """ Downloads a list of files from the cluster into the specified path
        on the local machine

        Parameters:
        files (List[str]): the list of files to copy to the cluster
        local_path (str): the local directory where the files will be put in
        remote_path (str): the cluster directory where the files currently reside in
        delete_sources (Boolean): if true, the source files on the cluster will be deleted. By default False
    """

    if isinstance(files, str):
        files = [files]

    logfolder = Path(config.get("DEFAULT", "LogFolder")).joinpath("transferC2P.log")
    script = current_path().joinpath("winscp/Cluster_to_PC.txt")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    putty_ssh_key = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.ppk")
    putty_ssh_key_passphrase = config.get("DEFAULT", "ClusterKeyPassword")

    if delete_sources:
        files.insert(0,"-delete")
    
    filestring = " ".join(files)
    
    print(f'Copying files {filestring} from cluster dir {remote_path} to local dir {local_path}...')
    return subprocess.run(
        [
            'winscp.com', '/ini=nul', f'/log={logfolder}', f'/script={script}', '/parameter',
            cluster_user,
            putty_ssh_key,
            putty_ssh_key_passphrase,
            local_path,
            remote_path,
            filestring
        ] 
    )

def download_files_with_retry(files: List[str], local_path: str, remote_path: str, delete_sources=False, max_tries = 300, interval = 10):
    """ Downloads a list of files from the cluster into the specified path
        on the local machine. In case of an error the operation will be 
        retried every {interval} seconds until {max_tries} tries are reached

        Parameters:
        files (List[str]): the list of files to copy to the cluster
        local_path (str): the local directory where the files will be put in
        remote_path (str): the cluster directory where the files currently reside in
        delete_sources (Boolean): if true, the source files on the cluster will be deleted. By default False
        max_tries (int): number of retries to attempt
        interval (int): Waiting interval in seconds between each retry
    """
    return_code = 1
    tries = 0
    while tries < max_tries:
        tries += 1
        completed_process = download_files(files, local_path, remote_path, delete_sources)
        return_code = completed_process.returncode
        if return_code == 0:
            break
        elif tries < max_tries:
            print(f'Cluster has not finished, retrying in {interval} seconds')
            time.sleep(interval)