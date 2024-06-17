# fk-firmware-isca

```
esptool.py --chip esp32 -p /dev/ttyUSB0 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 bootloader/bootloader.bin 0x10000 fk-isca.bin 0x8000 partition_table/partition-table.bin 0xd000 ota_data_initial.bin
```