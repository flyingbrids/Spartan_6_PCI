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

1. (Optional) - Create a desktop shortcut (make sure to replace 'USER_NAME' in the following lines with your username - as can be determined using the 'whoami' command)

    - Edit '~/Desktop/Xilinx-14.7.desktop' and add the following content:
      ```bash
      [Desktop Entry]
      Version=1.0
      Name=Xilinx ISE 14.7
      Exec=/home/USER_NAME/Xilinx/14.7/ISE_DS/ISE/bin/lin64/ise
      Terminal=false
      Icon=/home/USER_NAME/Xilinx/14.7/ISE_DS/ISE/data/images/pn-ise.png
      Type=Application
      Categories=Development
      ```
    - Change the permissions on the desktop shortcut
      ```bash
      chmod 777 ~/Desktop/Xilinx-14.7.desktop
      ```

# Building FPGA Code

The current development paradigm (using the Linux Test Application) requires generation of an Intel Hex format file that reflects the contents of the FPGA.  This can be done via the Xilinx 14.7 ISE (as described under the 'Development' section below) which generates a file named 'fpgautil_SourceFile.mcs', but the **'official'** mechanism for building the FPGA image is via make in this directory:
```bash
make clean
make
```
This will generate a file named '<build_timestamp>.intel_hex' in the 'build directory.

**NOTE** - The build process still uses the Xilinx project file ('SLX150.xise') to determine the VHDL files to build, so any modifications that change the list of files _should_ be done via the Xilinx ISE tool (though it is possible to manually edit the project file as it's an XML format file).  This was done so that development can be done via either mechanism.

**NOTE** - If you don't have a valid license, the build will fail at the 'Map' step.  To see the error, add 'VERBOSE=TRUE' to the 'make' command.  The cause of this error is due to an issue with either the step "Verify the name of the ethernet device used to generate the node-locked license" or "Obtain a [(free) node-locked license file".  Some causes to check:

1. Hostname for the license does not match the hostname of the machine (i.e. if machine name is 'host' and the license was generated for hostname 'hostt'.  This can be checked using the Unix 'hostname' command on the host and looking at the generated license information on the 'Manage Licenses' tab of the webpage (above) where you generated your license.
2. You don't have an ethernet device named 'ethX'.
3. The 'hostid' (on the 'Manage Licenses' tab) does not match the MAC address of the ethernet device (which can be determined by running the 'ifconfig' command on the host).
4. You _may_ have installed the wrong version (the correct one is the "Embedded Edition") of the ISE tool (in the "Execute the installation script." step above).


```bash
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
INFO:Security:56 - Part 'xc6slx150' is not a WebPack part.
INFO:Security:7 - A feature for ISE was found but is for the wrong hostid.
INFO:Security:61 - The XILINXD_LICENSE_FILE environment variable is not set.
INFO:Security:63 - The LM_LICENSE_FILE environment variable is not set.
INFO:Security:68 - For more information or for assistance in obtaining 
 a license, please run the Xilinx License Configuration Manager
       (xlcm or "Manage Xilinx Licenses".)
INFO:Security:68a - user is bsterling, on host unknown.
WARNING:Security:9b - No 'ISE' feature version 2013.10 was available for part
'xc6slx150'.
ERROR:Security:12 - No 'xc6slx150' feature version 2013.10 was available (-5),
     so 'WebPack' may not be used.
----------------------------------------------------------------------
Invalid host.
 The hostid of this system does not match the hostid
 specified in the license file.
Feature:       ISE
Hostid:        a0cec88ed831
License path: 
/home/bsterling/.Xilinx/Xilinx.lic:/home/bsterling/Xilinx/14.7/ISE_DS/ISE//data/
*.lic:/home/bsterling/Xilinx/14.7/ISE_DS/ISE//coregen/core_licenses/Xilinx.lic:/
home/bsterling/Xilinx/14.7/ISE_DS/ISE//coregen/core_licenses/XilinxFree.lic:
FLEXnet Licensing error:-9,57
For further information, refer to the FLEXnet Licensing documentation,
available at "www.flexerasoftware.com".No such feature exists.
Feature:       xc6slx150
License path: 
/home/bsterling/.Xilinx/Xilinx.lic:/home/bsterling/Xilinx/14.7/ISE_DS/ISE//data/
*.lic:/home/bsterling/Xilinx/14.7/ISE_DS/ISE//coregen/core_licenses/Xilinx.lic:/
home/bsterling/Xilinx/14.7/ISE_DS/ISE//coregen/core_licenses/XilinxFree.lic:
FLEXnet Licensing error:-5,357
For further information, refer to the FLEXnet Licensing documentation,
available at "www.flexerasoftware.com".
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ERROR:Map:258 - A problem was encountered attempting to get the license for this
   architecture.
```

## Known Warnings

There are (currently) two types of known warnings:

1.  WARNING:PhysDesignRules:367 - The signal <.../Mram_fifo_memory?_RAMD_D1_O> is incomplete. The signal does not drive any load pins in the design.

    This warning is generated by instantiating the entity defined in the 'generic_fifo.vhd' module and is due to the way the Xilinx tools internally generate memory.  Specifically, they generate memory blocks that handle (up to) 8 bits, but (for some reason), the tools only allocate 6 bits for each memory block.  So for each memory block allocated, two bits are unallocated, each resulting in a single warning.
2.  WARNING:NgdBuild:452 - logical net 'N???' has no driver

    The cause of this warning is currently unknown, but it is believed to be due to signals that were in the original Acromag designs but are no longer used.  There is currently no known way to determine where these 'logical nets' come from.

# (Xilinx ISE 14.7) Development Notes

This directory contains the latest FPGA code with the project file named 'SLX150.xise'.

Development can be done using any mechanism, but the following instructions reflect the use of the Xilinx 14.7 tool.

NOTE - when the Xilinx ISE project (SLX150.xise) is opened, it will automatically create a directory called 'iseconfig'.  It will also automatically create a 'build' directory if the user (as recommended) indicates at startup to do so.

## Building the FPGA image (from the Xilinx ISE tool)

1. Open the project in the Xilinx ISE.  Note that when you first open the project, it will indicate that there is no 'build' (working) directory.  Select 'Create' to create this directory.
1. Make sure that 'View:' is set to 'Implementation' in the 'Design' panel
1. In the 'Hierarchy' pane, select 'XC6SLX150 - XC6SLX150_ARCH (XC5SLX150.vhd)' under 'xc6slx150-3fgg676'
1. In the 'Processes' pane, double (left) click on 'Configure Target Device'.  This will run all processes necessary and then open a Dialog regarding 'iMPACT'.
    ![Impact Warning](../Documentation/ImpactWarning.png)
1. Select 'OK' to open the iMPACT tool.
1. Double click 'Generate File...' in the 'iMPACT Processes' section.  This will create a file named 'fpgautil_SourceFile.mcs' in the 'Fpga/build' directory.  NOTE that there is no way to automatically generate the .mcs file from the tool (but it can be generated automatically by using the command line - this is not currently documented).
1. Close the iMPACT tool.  This will open a dialog asking whether or not to save the Project file.  This is a no-op as the project has not changed.

The FPGA image will be in a file named 'fpgautil_SourceFile.mcs' in the 'build' subdirectory

## Testing the UART

1. Open the project in the Xilinx ISE
1. In 'View:', select 'Simulation'
1. In the 'Hierarchy' pane, select 'xc6slx150-3fgg676-> tb_uart - vhdl (tb_uart.vhd)'
1. In the 'Processes' pane, double (left) click on 'ISim Simulator-> Simulate Behavioral Model'.  This will bring up ISim.
1. The default run time in ISim is 1.00us, which is not enough time for the entire simulation to complete.  As such, select 'Simulation-> Run All' to finish the simulation.
1. Verify that 'build/output.txt' has the exact same contents as 'uart/input.txt'

## Adding new, pre-existing VHDL files

To add a new VHDL source file:
1. In the 'Hierarchy' pane, select 'XC6SLX150 - XC6SLX150_ARCH (XC5SLX150.vhd)' under 'xc6slx150-3fgg676'
   1. Select 'Project-> Add Source'

## Unconsolidated Notes

### Specifying the '.mcs' file generation

The 'BitStreamGenerator.ipf' file is generated by the iMPACT tool in order to store the configuration information for generating the '.mcs' file.  The following is the process that was used in the iMPACT tool in order to create the configuration.

1. Double (left) click on 'Create PROM File (PROM File Formatter)
1. Select 'Generic Parallel PR...' and click the (green) arrow
1. Select 'Auto Select PROM', select the largest PROM size, and click the second (green) arrow
1. In the 'Output File Name' box, enter 'fgpautil_SourceFile'
1. Make sure 'File Format' is set to 'MCS'
1. Click 'OK'
1. In the 'Add Device' dialog, select the 'XC6SLX150.bit' file
1. Select 'No' (to adding another device)
1. Select 'OK' to continue
1. Select 'OK' to accept the default addresses

### Code Version History

Per the 'README.md' file in the 'working' directory as of 2023_09_21, the original FPGA effort was based on the 256K version of the X6SLX150 board (X6SLX150_9000601).  Furthermore, the information available at that time indicated that version 13.2 of the tools was the only one that worked for the hardware.  As such, this directory was originally populated using <distribution_cd>/ISE Project Files/AXM_D02_D03_EDK/VHDL/X6SLX150_9000601/ as the source (this was the _ONLY_ 13.2 version project file for the  256K version of the board).

