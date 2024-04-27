'''
    Get the latest log from the raspberry
'''
import paramiko
from subprocess import call
from tqdm import tqdm
from pathlib import Path
from .utils import sorted_nicely

last_value = 0

def create_tqdm_callback(*args, **kwargs):
    """Instanciate a tqdm bar and return a callback for paramiko"""
    # Instanciate tqdm bar once
    pbar = tqdm(*args, **kwargs)
    # Create the actual callback
    def viewBar(a, b):
        """Update callback: update total and n (current iteration)"""
        global last_value
        pbar.total = int(b)
        pbar.update(a - last_value)
        last_value = a
    # Return the callback
    return viewBar

def main():
    sftpURL   =  '192.168.6.2'
    sftpUser  =  'pi'
    sftpPass  =  'raspberry'

    ssh = paramiko.SSHClient()
    # automatically add keys without requiring human intervention
    ssh.set_missing_host_key_policy( paramiko.AutoAddPolicy() )

    ssh.connect(sftpURL, username=sftpUser, password=sftpPass)

    ftp = ssh.open_sftp()
    ftp.chdir("logs/")
    files = ftp.listdir()
    files = sorted_nicely(files)

    viewBar = create_tqdm_callback(unit='b', unit_scale=True)
    ftp.get(files[-1], "./" + files[-1], callback = viewBar)
    del viewBar

    call(f"miam_log_converter {files[-1]}", shell=True)
    print(f"Copied '{Path(files[-1]).with_suffix('.hdf5')}' from the robot to the current working directory")

if __name__ == "__main__":
    main()