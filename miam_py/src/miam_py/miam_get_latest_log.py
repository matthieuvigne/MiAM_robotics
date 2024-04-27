'''
    Get the latest log from the raspberry
'''
import paramiko
from subprocess import call
from .utils import sorted_nicely

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

    ftp.get(files[-1], "./" + files[-1])
    call(f"miam_log_converter {files[-1]}", shell=True)
    print(f"Copied '{files[-1]}' from the robot to the current working directory")

if __name__ == "__main__":
    main()