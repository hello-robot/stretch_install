# Configuring the BIOS

This documentation describes how to configure the BIOS of an Intel NUC for compatibility with the stretch installation procedure.

# Accessing the NUC BIOS Settings

First plug in the NUC to a 19V DC power supply. Next power on the NUC using the power button on the fron of the NUC.

When powered on, a NUC with factory settings, should display a welcome screen sBIOS access instructions similar to the picture below:

![](./images/NUC_startup.png)

When this label becomes visible press 'F2' to enter into the BIOS configuration menu.

The BIOS Settings page should look like the picture below:

![](./images/BIOS_settings.png)

Select the 'Advanced' drop down menu near the top right of the screen, and then slect the option 'Boot'

![](./images/BIOS_advanced.png)

From the 'Boot' settings page select the 'Secure Boot' tab.

![](./images/BIOS_secure_boot_tab.png)

Turn off 'Secure Boot' by toggling the checkbox labeled 'Secure Boot' to unchecked.


![](./images/BIOS_secure_boot_check.png)

Next Select the 'Power' tab


![](./images/BIOS_power_tab.png)

From the power settings screen select the 'Power On' option from the 'After Power Failure' drop down selection.


![](./images/BIOS_power_settings.png)


Next Select the Security tab

![](./images/BIOS_security_tab.png)


Turn on UEFI third party drivers compatibility by toggling the checkbox labeled 'Allow UEFI Third Party Driver loaded' to checked.

![](./images/BIOS_security_settings.png)


Now use the F10 key to save BIOS configuration changes and exit.


