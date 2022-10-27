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
        print(f'Connecting to {cluster_host} ...')
        logger.debug("Connecting to cluster...")
        ssh.connect(hostname=cluster_host, username=cluster_user, pkey=ssh_key)
        print(f'Connection established')
        logger.debug("Connection established")

        print(f'Executing command...')
        logger.debug("Executing command...")
        print(cmd)
        _, stdout, _ = ssh.exec_command(cmd)
        output = stdout.readlines()
        exit_code = stdout.channel.recv_exit_status()
        if exit_code != 0:
            logger.exception("Could not execute command")
            raise Exception(f'Could not execute command: ', output)
    finally:
        ssh.close()
        print("Connection closed")
        logger.debug("Command sent, connection closed")
        logger.debug("Finished function send_command")
