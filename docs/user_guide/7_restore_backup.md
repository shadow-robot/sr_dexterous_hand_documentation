# Shadow Backup USB stick

You can only follow these steps if you have been provided with a Shadow Backup USB stick. It has a 500 MB Clonezilla partition and a several GB for Clonezilla disk images. These can be used to restore any Shadow-provided laptop(s) and/or NUC(s) to their exact configuration (it's a full disk image restore) after the delivery was tested in Shadow's offices but before it was shipped to the customer.

**Doing this means losing all the data on the laptop(s) and/or NUC(s) since the original shipment of the equipment from Shadow. Please back up your data to an external device before restoring any Shadow backups**
 
 If you a restoring a laptop backup, you only need the Shadow backup USB stick.
 
 If you are restoring a NUC backup, you will need additional equipment (not provided by Shadow): A monitor with HDMI connector, an HDMI cable and a USB keyboard.
 
 The steps for restoring a NUC with Shadow Backup USB stick are the same as for laptop, but you have to choose a different image to restore from the relevant list in the Clonezilla process. 

 You don't have to restore both NUC and laptop. You can choose to restore just 1 of them.

## Clonezilla backup restore steps (here device means either laptop or NUC)

1. Connect the Shadow USB stick to the device you want to restore

2. Power on the device while tapping F7 and F8 on the keyboard. A blue dialog box will appear asking you which disk to boot from (if this doesn't work, try tapping F2/F12 and accessing the boot menu/boot override via BIOS): select the Shadow backup USB 500 MB Clonezilla partition (it might show up as a SanDisk device, or a similar brand matching the model of the USB stick)

3. A Clonezilla window will appear

    ```eval_rst
    .. image:: ../img/clonezilla_1.png
    ```

4. Select Clonezilla live (Default settings, VGA 1024x768)

5. In the next menu, select English for the language: (just press Enter)

    ```eval_rst
    .. image:: ../img/clonezilla_2.png
    ```

6. In the next screen, the default keyboard layout is US keyboard, and it’s fine for our purposes here, so just press Enter

    ```eval_rst
    .. image:: ../img/clonezilla_3.png
    ```

7. Select Start Clonezilla

    ```eval_rst
    .. image:: ../img/clonezilla_4.png
    ```

8. Choose device-image

    ```eval_rst
    .. image:: ../img/clonezilla_5.png
    ```

9. Choose local device:

    ```eval_rst
    .. image:: ../img/clonezilla_6.png
    ```

10. Press enter when you see this screen:

    ```eval_rst
    .. image:: ../img/clonezilla_7.png
    ```

11. Press Ctrl-C when you see this screen:

    ```eval_rst
    .. image:: ../img/clonezilla_8.png
    ```

12. You should see a disk menu like this (not the exact image), but you need to select your Shadow Backup USB stick large partition with several GB (where the disk images are) (it might show up as a SanDisk device, or a similar brand matching the model of the USB stick, but with several GB of space)

    ```eval_rst
    .. image:: ../img/clonezilla_9.png
    ```

13. In the folder selection screen below you should see 2 folders for the clonezilla images for NUC and laptop, don’t select them, just select Done

    ```eval_rst
    .. image:: ../img/clonezilla_10.png
    ```

14. Choose Beginner mode

    ```eval_rst
    .. image:: ../img/clonezilla_11.png
    ```

15. Choose restoredisk option

    ```eval_rst
    .. image:: ../img/clonezilla_12.png
    ```

16. Now is the time to select whether you want to restore a NUC image or a laptop image. Depending on which device you have connected the Shadow backup USB stick to, select either the NUC image (may be labelled with your customer name and NUC or NUC-CONTROL and 256GB) or the laptop image (may be labelled with your customer name and LAPTOP or SERVER and 500 GB)

    ```eval_rst
    .. image:: ../img/clonezilla_13.png
    ```

17. Just press enter on the destination disk:

    ```eval_rst
    .. image:: ../img/clonezilla_14.png
    ```

18. Select “No, skip checking the image before restoring”

    ```eval_rst
    .. image:: ../img/clonezilla_15.png
    ```

19. Select -p choose

    ```eval_rst
    .. image:: ../img/clonezilla_16.png
    ```

20. Press enter:

    ```eval_rst
    .. image:: ../img/clonezilla_17.png
    ```

21. There will be 2 prompts and you have to press y and press Enter to each of them

    ```eval_rst
    .. image:: ../img/clonezilla_18.png
    ```

22. Clonezilla is now restoring the device image:

    ```eval_rst
    .. image:: ../img/clonezilla_19.png
    ```

23. It should take about 20 minutes or less. When Clonezilla is done, press Enter:

    ```eval_rst
    .. image:: ../img/clonezilla_20.png
    ```

24. Choose poweroff

    ```eval_rst
    .. image:: ../img/clonezilla_21.png
    ```

25. Unplug the Shadow Backup USB stick from the device

26. Power on the device. The device is now restored.
