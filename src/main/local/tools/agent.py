import os
from pathlib import Path
import paramiko
import sys

project_path = sys.argv[1]
cluster_user = sys.argv[2]
key_passwd = sys.argv[3]
ssh_key_folder = sys.argv[4]

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)

ssh.connect("login18-1.hpc.itc.rwth-aachen.de", username=cluster_user, pkey=paramiko.Ed25519Key.from_private_key_file(Path(ssh_key_folder).joinpath("key.priv"), password=key_passwd))
print("HPC CLUSTER: connected")

print("Submitting job")
stdin, stdout, stderr = ssh.exec_command('cd /home/wj777230/Dokumente/topologyoptimizer && sbatch agent.job') # Submit job
print(stdout.readlines())
print("Job started")

ssh.close()
print("HPC CLUSTER: connection closed")