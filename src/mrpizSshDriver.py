#!/usr/bin/env python
# Version : 0.1
# Date : 02/03/2018
# Mace Robotics

import paramiko

# definition des informations :
hostname = '192.168.42.2' # IP du robot MRPiZ
password = 'raspberry'
username = 'pi'
port = 22

ssh = paramiko.SSHClient()

# connexion SSH au robot MRPiZ
def connexionToMRPiZ():  
  ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
  ssh.load_system_host_keys()
  ssh.connect(hostname=hostname, port=port, username=username, password=password)
  print "connexion to MRPiZ is open."

def connexionToMRPiZClose():  
  ssh.close()
  print "connexion to MRPiZ is close."

# move MRPiZ robot
def ssh_forward(speed):
  command = 'echo "#MF,' + str(speed) + '!" > /dev/ttyAMA0'
  ssh.exec_command(command) # commande SSH

def ssh_stop():
  command = 'echo "#STP!" > /dev/ttyAMA0'
  ssh.exec_command(command) # commande SSH

def ssh_back(speed):
  command = 'echo "#MB,' + str(speed) + '!" > /dev/ttyAMA0'
  ssh.exec_command(command) # commande SSH

def ssh_turnLeft(speed):
  command = 'echo "#TL,' + str(speed) + '!" > /dev/ttyAMA0'
  ssh.exec_command(command) # commande SSH

def ssh_turnRight(speed):
  command = 'echo "#TR,' + str(speed) + '!" > /dev/ttyAMA0'
  ssh.exec_command(command) # commande SSH

def ssh_proximity(sensor_number):
  command = 'python -c "from mrpiZ_lib import * \rprint proxSensor(' + str(sensor_number) + ')"'
  stdin, stdout, stderr = ssh.exec_command(command) # commande SSH
  value =  stdout.channel.recv(4)
  return float(value)


def ssh_robotPositionX():
  command = 'python -c "from mrpiZ_lib import * \rprint robotPositionX()"'
  stdin, stdout, stderr = ssh.exec_command(command) # commande SSH
  position =  stdout.channel.recv(4)
  return float(position)

def ssh_robotPositionY():
  command = 'python -c "from mrpiZ_lib import * \rprint robotPositionY()"'
  stdin, stdout, stderr = ssh.exec_command(command) # commande SSH
  position =  stdout.channel.recv(4)
  return float(position)

def ssh_robotPositionO():
  command = 'python -c "from mrpiZ_lib import * \rprint robotPositionO()"'
  stdin, stdout, stderr = ssh.exec_command(command) # commande SSH
  orientation =  stdout.channel.recv(4)
  return float(orientation)

def ssh_resetUc():
  command = 'echo "#RST!" > /dev/ttyAMA0'
  ssh.exec_command(command) # commande SSH


