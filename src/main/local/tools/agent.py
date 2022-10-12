import os
import paramiko
import sys

project_path = sys.argv[1]
cluster_user = sys.argv[2]
key_passwd = sys.argv[3]
cluster_path = sys.argv[4]
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)

ssh.connect("login18-1.hpc.itc.rwth-aachen.de", username=cluster_user, pkey=paramiko.Ed25519Key.from_private_key_file(os.path.join(project_path, "tools", "Keys", "key.priv"), password=key_passwd))
print("HPC CLUSTER: connected")

print("Submitting job")
stdin, stdout, stderr = ssh.exec_command('cd /home/'+ cluster_user +'/'+ cluster_path +'/src/main/cluster && sbatch agent.job') # Submit job
print(stdout.readlines())
print("Job started")

ssh.close()
print("HPC CLUSTER: connection closed")