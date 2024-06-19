Scrapes temperature from an InkBird IBS-P01B Bluetooth pool floaty.

Characteristic 0x23 stores temperature as little endian 2-byte signed int, as degrees_C * 100
* This is attribute e.g. `/org/bluez/hci0/dev_49_24_01_10_04_C0/service001f/char0023` in `bluetoothctl`
* Device name is `tps`
* MAC for my unit is `49:24:01:10:04:C0`

Various github projects use different characteristic IDs; one is 0x24, another
is 0x28. Mine is 0x23. Presumably this is due to software or hardware revs. That said, the githubs I
found were using Python on a raspi, which is using hex IDs instead of UUIDs like a normal BT API. This
code uses UUIDs, which seem to be more stable. Specifically, 0xfff0 is the service UUID16, and 0xfff2 is
the specific characteristic UUID16.

`bluetoothctl` can be used to inspect a given device to find out authoritatively for that device:

1. `bluetoothctl`
2. `scan on`
3. Wait for device to appear, and harvest its MAC. The device ID string does seem to uniformly be "tps", at least.
4. `connect {MAC}`
5. `menu gatt`
6. `select-attribute {TAB}{TAB}`
7. Find the primary service (mine is `service001f`) 
8. Check the descriptor value for each characteristic, i.e. `.../char{XXXX}/desc{YYYY}`
9. `read` to print the value. The descriptor that says "Real time data" is the one containing the data.
10. `select-attribute .../char{XXXX}` to switch to the actual data field of the characteristic
11. `read` to print the hex value; doing the binary ops to reconstruct it as an integer should yield a value which matches display
12. Once the right characteristic is located and confirmed, note its UUID16 and that of its service.
