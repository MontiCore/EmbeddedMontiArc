from ast import List
from pathlib import Path
import paramiko
from config import config


def schedule_job(job_name: str, job_file: str):
    cluster_host = config.get("DEFAULT", "ClusterHost")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    cluster_working_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    cluster_project_account = config.get("DEFAULT", "ClusterProjectAccount")
    cluster_log_folder = config.get("DEFAULT", "ClusterLogFolder")
    job_file_folder = Path(cluster_working_dir).joinpath("jobs").as_posix()
    job_filename = Path(job_file).name
    ssh_key_file = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.priv")
    ssh_key_passwd = config.get("DEFAULT", "ClusterKeyPassword")
    ssh_key = paramiko.Ed25519Key.from_private_key_file(ssh_key_file, ssh_key_passwd)

    options = [
        f'--job-name="{job_name}"',
        f'--output="{cluster_log_folder}/%x_%j.log"',
        f'--account="{cluster_project_account}"',
        f'--export="WORKING_DIR={cluster_working_dir}"'
    ]
    optionsstring = " ".join(options)
    
    cmd = f'cd {job_file_folder} && sbatch {optionsstring} {job_filename} 2>&1'

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)
    try:
        

        print(f'Connecting to {cluster_host} ...')
        ssh.connect(hostname=cluster_host, username=cluster_user, pkey=ssh_key)
        print(f'Connection established')

        print(f'Submitting job {job_name}')
        print(cmd)
        _, stdout, _ = ssh.exec_command(cmd)
        output = stdout.readlines()
        exit_code = stdout.channel.recv_exit_status()
        if exit_code != 0:
            raise Exception(f'Could not schedule job', "\n".join(output))
    finally:
        ssh.close()
        print("Connection closed")