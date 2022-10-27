import logging
from pathlib import Path
from config import config
import paramiko

# Initialize logging
logger = logging.getLogger(__name__)

def send_command(cmd):
    logger.debug("Start function send_command")
    cluster_host = config.get("DEFAULT", "ClusterHost")
    cluster_user = config.get("DEFAULT", "ClusterUser")
    ssh_key_file = Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.priv")
    ssh_key_passwd = config.get("DEFAULT", "ClusterKeyPassword")
    ssh_key = paramiko.Ed25519Key.from_private_key_file(ssh_key_file, ssh_key_passwd)

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy)

    try:
        logger.debug("Connecting to host %s...", cluster_host)
        ssh.connect(hostname=cluster_host, username=cluster_user, pkey=ssh_key)
        logger.debug("Connection established")

        logger.debug("Executing command '%s'...",cmd)
        _, stdout, _ = ssh.exec_command(cmd)
        output = stdout.readlines()
        exit_code = stdout.channel.recv_exit_status()
        if exit_code != 0:
            raise Exception(f'Could not execute command: ', output)
    finally:
        ssh.close()
        logger.debug("Connection closed")
        logger.debug("Finished function send_command")
