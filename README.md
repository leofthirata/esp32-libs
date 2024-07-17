# fk-firmware-isca

- edit sdkconfig
    ```
    CONFIG_FREERTOS_HZ=1000
    ```
- run `idf.py  menuconfig` and enable ppp support
    ```
    Component config  / Enable PPP support
    ```
- flashing using esptool
    ```
    esptool.py --chip esp32 -p /dev/ttyUSB0 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 bootloader/bootloader.bin 0x10000 fk-isca.bin 0x8000 partition_table/partition-table.bin 0xd000 ota_data_initial.bin
    ```

# Tasks intercommunication
- using the defaul event loop

```mermaid
sequenceDiagram
  participant loraTask
  participant stateTask
  participant gsmTask

loop p2pTImer
    stateTask->>+loraTask: P2P_TX_REQ
    loraTask->>-stateTask: P2P_TX_RES
    Note over stateTask,loraTask: uses LoRaP2PReq_t
    Note left of loraTask: P2P
    opt if p2p received
    loraTask-->>stateTask: P2P_RX
    Note over stateTask,loraTask: uses LoRaP2PRx_t
    end
end

loop lrwTimer
    stateTask->>+loraTask: LRW_TX_REQ
    Note over stateTask,loraTask: uses LoRaLRWTxReq_t
    loraTask->>-stateTask: LRW_TX_RES
    Note over stateTask,loraTask: uses LoRaLRWTxRes_t
        Note left of loraTask: LRW
    opt if lrw received
    loraTask-->>stateTask: LRW_RX
    Note over stateTask,loraTask: uses LoRaLRWRx_t
    end
end
loop gsmTimer
    stateTask->>+gsmTask: GSM_TX_REQ
    Note over stateTask,gsmTask: uses GSMTxReq_t
    gsmTask->>-stateTask: GSM_TX_RES
    Note over stateTask,gsmTask: uses GSMTxRes_t
    Note right of gsmTask: GSM
    opt if gsm received
    gsmTask-->>stateTask: GSM_RX
    Note over stateTask,gsmTask: uses GSMRx_t
    end
end  

```

## OTP Organization

- 9 pages of 8 bytes;
- the 8th byte [bit 7] of each page is a CRC;

![alt text](docs/OTPMemory.png)

- Example:
    ```
    Slave Address = 110b
        Standard Speed ACK
        Data Read = 0x01 0x00 0x6D 0xBE 0xBC 0x20 0x17 0x9C
        Data Read = 0x9F 0x2B 0xBA 0xD7 0x3F 0x71 0x26 0x18
        Data Read = 0xE8 0x77 0x6C 0xCE 0xBF 0x6E 0xAA 0xFB
        Data Read = 0x8F 0xCE 0x08 0xED 0xB9 0x02 0xE9 0x65
        Data Read = 0xBA 0xCE 0x93 0xD9 0x71 0x3C 0x4E 0x92
        Data Read = 0x17 0x41 0x93 0x55 0x49 0x95 0x97 0xBB
        Data Read = 0xD9 0x1B 0xC3 0x52 0x7F 0x9C 0xDB 0x3E
        Data Read = 0xBA 0x4F 0x32 0x5F 0x2E 0x62 0x38 0xCB
        Data Read = 0x0F 0xCF 0x00 0x00 0x00 0x00 0x00 0xF9
        [PARSE] memVer: 1 | hwVer: 0 | prefixSN: 109 
            loraID: 0xBE 0xBC 0x20 = 12500000
            devAddress: 0x17 0x9F 0x2B 0xBA
            devEUI: 0xD7 0x3F 0x71 0x26 0xE8 0x77 0x6C 0xCE
            appEUI: 0xBF 0x6E 0xAA 0x8F 0xCE 0x08 0xED 0xB9
            nwSKey: 0x02 0xE9 0xBA 0xCE 0x93 0xD9 0x71 0x3C 0x4E 0x17 0x41 0x93 0x55 0x49 0x95 0x97
            appSKey: 0xD9 0x1B 0xC3 0x52 0x7F 0x9C 0xDB 0xBA 0x4F 0x32 0x5F 0x2E 0x62 0x38 0x0F 0xCF
    ```

## GSM Packet