import asyncio
from asyncio.subprocess import PIPE
import logging
import random
import subprocess
from pathlib import Path
import time
from typing import List
from config import config
import paramiko

# Initialize logging
logger = logging.getLogger(__name__)

def current_path():
    return Path( __file__ ).parent.resolve()

async def upload_files(files: List[str], local_path: str, remote_path: str, delete_sources = False):
    """ Uploads a list of files to the cluster into the specified path

        Parameters:
        files (List[str]): the list of files to copy to the cluster
        remote_path (str): the directory where to copy the files to
    """
    logger.debug("Start function upload_files")
    if isinstance(files, str):
        files = [files]

    logfolder = Path(config.get("DEFAULT", "LocalLogFolder")).joinpath("transferP2C.log")
    script = current_path().joinpath("winscp/PC_to_Cluster.txt")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    putty_ssh_key = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.ppk")
    putty_ssh_key_passphrase = config.get("DEFAULT", "ClusterKeyPassword")

    options = []

    if delete_sources:
        options.append("-delete")
    
    filestring = f'{" ".join(options)} {" ".join(files)}'

    logger.debug("Copying files to cluster...")
    process = await asyncio.create_subprocess_exec('winscp.com',
                                                   '/ini=nul',
                                                   f'/log={logfolder}',
                                                   f'/script={script}',
                                                   '/parameter',
                                                   cluster_user,
                                                   putty_ssh_key,
                                                   putty_ssh_key_passphrase,
                                                   local_path,
                                                   remote_path,
                                                   filestring,
                                                   stdout=PIPE,
                                                   stderr=PIPE
    )

    stdout, stderr = await process.communicate()

    return process.returncode


async def download_files(files: List[str], local_path: str, remote_path: str, delete_sources=False):
    """ Downloads a list of files from the cluster into the specified path
        on the local machine

        Parameters:
        files (List[str]): the list of files to copy to the cluster
        local_path (str): the local directory where the files will be put in
        remote_path (str): the cluster directory where the files currently reside in
        delete_sources (Boolean): if true, the source files on the cluster will be deleted. By default False
    """

    logger.debug("Start function download_files")
    if isinstance(files, str):
        files = [files]

    logfolder = Path(config.get("DEFAULT", "LocalLogFolder")).joinpath("transferC2P.log")
    script = current_path().joinpath("winscp/Cluster_to_PC.txt")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    putty_ssh_key = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.ppk")
    putty_ssh_key_passphrase = config.get("DEFAULT", "ClusterKeyPassword")

    options = []

    if delete_sources:
        options.append("-delete")
    
    filestring = f'{" ".join(options)} {" ".join(files)}'
    
    logger.debug(f'Copying files {filestring} from cluster dir {remote_path} to local dir {local_path}...')
    process = await asyncio.create_subprocess_exec('winscp.com', 
                                                    '/ini=nul',
                                                    f'/log={logfolder}',
                                                    f'/script={script}',
                                                    '/parameter',
                                                    cluster_user,
                                                    putty_ssh_key,
                                                    putty_ssh_key_passphrase,
                                                    local_path,
                                                    remote_path,
                                                    filestring,
                                                    str(random.randint(1, 1000000)),
                                                    stdout=PIPE,
                                                    stderr=PIPE
    )

    stdout, stderr = await process.communicate()
    
    return process.returncode

async def download_files_with_retry(files: List[str], local_path: str, remote_path: str, delete_sources=False, max_tries = 300, interval = 10):
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
    logger.debug("Start function download_files_with_retry")
    return_code = 1
    tries = 0
    try:
        while tries < max_tries:    
            tries += 1
            return_code = await download_files(files, local_path, remote_path, delete_sources)
            if return_code == 0:
                logger.debug("Files successfully downloaded")
                break
            elif tries < max_tries:
                logger.debug(f'Attempt {tries}/{max_tries}, files still not available. Retry in {interval} seconds')
                await asyncio.sleep(interval)
        logger.debug(f'Files still not available after {max_tries} tries, aborting...')
    except asyncio.CancelledError:
        raise


def exists(remote_file: str):
    logger.debug("Start function exists")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    ssh_key_password = config.get("DEFAULT", "ClusterKeyPassword")
    ssh_key_file = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.priv")
    ssh_key = paramiko.Ed25519Key.from_private_key_file(ssh_key_file, ssh_key_password)
    hostname = config.get("DEFAULT", "ClusterHost")

    # the command to run remotely to check if the file exists
    cmd = f'ls -a -d {remote_file} > /dev/null 2>&1 || exit $?'
    
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)

    print(f'Connecting to {hostname} ...')
    logger.debug("Connecting to cluster...")
    try:
        ssh.connect( hostname=hostname, username=cluster_user, pkey=ssh_key)
        
        print(f'Connection established')
        logger.debug("Connection established")

        print(f'Executing remote command "{cmd}"')
        logger.debug("Checking if file exists...")
        _, stdout, _ = ssh.exec_command(cmd)
        if stdout.channel.recv_exit_status() == 0:
            return True
        else:
            return False
    finally:
        ssh.close()
        print("Connection closed")
        logger.debug("Connection closed")
        logger.debug("Finished function exists")

def mkdir(remote_dir: str):
    logger.debug("Start function mkdir")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    ssh_key_password = config.get("DEFAULT", "ClusterKeyPassword")
    ssh_key_file = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.priv")
    ssh_key = paramiko.Ed25519Key.from_private_key_file(ssh_key_file, ssh_key_password)
    hostname = config.get("DEFAULT", "ClusterHost")

    # the command to run remotely to check if the file exists
    cmd = f'mkdir -p {remote_dir} 2>&1 || exit $?'
    
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)

    print(f'Connecting to {hostname} ...')
    logger.debug("Connecting to cluster...")
    try:
        ssh.connect( hostname=hostname, username=cluster_user, pkey=ssh_key)
        print(f'Connection established')
        logger.debug("Connection established")

        print(f'Executing remote command "{cmd}"')
        logger.debug("Creating remote directory")
        _, stdout, _ = ssh.exec_command(cmd)
        output = stdout.read()
        exit_code = stdout.channel.recv_exit_status()
        if exit_code != 0:
            logger.exception("Could not create directory on the cluster")
            raise Exception(f'Could not create {remote_dir} on the cluster', output)
    finally:
        ssh.close()
        print("Connection closed")
        logger.debug("Connection closed")
        logger.debug("Finished function mkdir")

async def synchronize_directory(local_dir: str, remote_dir: str):
    logger.debug("Start function synchronize_directory")
    logfolder = Path(config.get("DEFAULT", "LocalLogFolder")).joinpath("transferC2P.log")
    script = current_path().joinpath("winscp/Synchronize_directory.txt")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    putty_ssh_key = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.ppk")
    putty_ssh_key_passphrase = config.get("DEFAULT", "ClusterKeyPassword")    

    process = await asyncio.create_subprocess_exec(
            'winscp.com', '/ini=nul', f'/log={logfolder}', f'/script={script}', '/parameter',
            cluster_user,
            putty_ssh_key,
            putty_ssh_key_passphrase,
            local_dir,
            remote_dir
    )

    await process.communicate()
        
    return process.returncode