'''
    Get the latest log from the raspberry
'''
import paramiko

import re

def sorted_nicely( l ):
    """ Sort the given iterable in the way that humans expect."""
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)


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
    print(f"Copied '{files[-1]}' from the robot to the current working directory")

if __name__ == "__main__":
    main()