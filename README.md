# Generate SSH Key
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

# Add SSH Key to the settings of the github 
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account

use cat command to copy the generated .pub key to the clipboard

# Cloning
To clone the repository, you either need to set up SSH, or clone via HTTPS as follows:
    git clone git@github.com:flyingbrids/Spartan_6_PCI.git --config core.sshCommand="ssh -i ~/location/to/private_ssh_key"

# Pushing 


# Xilinx ISE 14.7 Setup

If working directly with the FPGA (i.e. the device drivers are needed in order to program the FPGA), the install (i.e. './xsetup') would have to be done using 'sudo', and 'sudo' must have privileges to install device drivers.  **However**, because programming is done indirectly, there is no need to install the device drivers, and therefore no need to run the install program as 'sudo' (just make sure the 'cable' item is unselected during the install).  The following instructions assume that the install is not being done using 'sudo'.

1. Install the pre-requisite libncurses package
   ```bash
   sudo apt install libncurses5
   ```
1. Verify the name of the ethernet device used to generate the node-locked license
   Some (but not all) of the Xilinx tools used require the ethernet device whose MAC address is used to generate the (free) node-locked license to have a specific name.  If this requirement is not met, the 'Implement Design' feature in the Xilinx tools will fail with a cryptic message at the 'Map' step.  It is unknown what all of the possible valid names are, but it is known that 'eth0' is one of them.  As such, if no such device already exists on the host system, either create a dummy ethernet device or rename an existing ethernet device.
    1. Rename an existing ethernet device
       There are a number of ways to achieve this.  One way is via a new netplan:
        1. Run 'ifconfig' and select a ethernet device to rename.  For instance, 'enxa0cec88ed831'.
        1. Create a new file in the /etc/netplan directory to rename the existing interface.  For instance, '/etc/netplan/02-ethernets.yaml':
            ```bash
            network:
              ethernets:
                enxa0cec88ed831:
                  match:
                    macaddress: a0:ce:c8:8e:d8:31
                  set-name: eth0
                  dhcp4: true
                  optional: true
            ```
    1. Create a dummy ethernet device
        ```bash
        sudo modprobe dummy
        sudo ip link add eth0 type dummy address 11:22:33:44:55:66
        sudo ip link set dev eth0 address 1a:2b:3c:4d:5e:6f
        sudo ifconfig eth0 up
        ```
1. Obtain a [(free) node-locked license file](https://www.xilinx.com/member/forms/license-form.html) (You'll need an AMD account).
    - Select 'Xilinx MicroBlaze/All Programmable SoC Software Development Kit - Standalone'
    - Select 'ISE WebPACK License'
    - Select 'ISE Embedded Edition License'
    - Select '2023 AI Engine Tools License'
    - Select 'Vitis HLS License'
    - Select 'Generate Node-Locked License'
        - Select a host... -> Add a host... ->
             - Operating System: **Linux 64-bit**
             - Host Name: **<enter the value returned from the command line 'hostname' command>**
             - Host ID Type: **Ethernet MAC**
             - Host ID Value: **<enter the MAC address for the ethernet device in the step above>**
    - An email containing the license will be sent to the registered email address.  Save this file on your local system as 'Xilinx.lic' in directory '$HOME/.Xilinx' (this is one of the directories automatically searched by the programs to find valid licenses).
1. [Download the 14.7 ISE](https://www.xilinx.com/member/forms/download/xef.html?filename=Xilinx_ISE_DS_Lin_14.7_1015_1.tar) (You'll need an AMD account)
1. Untar the downloaded file
   ```bash
   tar xvf Xilinx_ISE_DS_Lin_14.7_1015_1.tar
   ```
1. Execute the installation script.
   ```bash
   cd Xilinx_ISE_DS_Lin_14.7_1015_1
   ./xsetup
   ```
   This will launch the interactive setup program.  The following are the steps necessary to install:

    - Next >
    - Accept both agreements
    - Next >
    - Accept the license agreements
    - Next >
    - Select 'ISE Design Suite: Embedded Edition'
    - Next >
    - Select 'Acquire or Manage a License Key'
    - Unselect 'Enable WebTalk to send software, IP and device usage statistics to Xilinx (Always enabled for WebPACK license)'
    - Make sure 'Install Cable Drivers' is not selected (this requires 'sudo' privileges)
    - Next >
    - Change the installation directory using the 'Browse...' button from '/opt/Xilinx - which would require 'sudo' privileges - to '$HOME/Xilinx'
    - Unselect 'Import tool preferences from previous...'
    - Next >
    - Install
    - Finish




