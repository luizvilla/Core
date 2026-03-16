# Minimal RS485 MASTER/FOLLOWER 

This is a minimal two-board RS485 example.

- One board is `ROLE_MASTER`
- One board is `ROLE_FOLLOWER`
- MASTER sends `voltage_ref`
- FOLLOWER replies with `voltage_meas`

## Minimal hardware setup

![](Images/schema_MF_TWIST.png)

Connect two TWIST with a RJ45 cables, and supply the two TWIST with at least 12V.

## Role selection (manual)

Role is selected by changing one variable in `main.cpp`:

```cpp
uint8_t role_id = ROLE_MASTER;
```

Set one board to `ROLE_MASTER` and the other to `ROLE_FOLLOWER`.

## Frame format

```cpp
struct RS485_frame
{
    float voltage_ref;
    float voltage_meas;
    uint8_t sender_id;
    uint8_t status_code;
} __packed;
```
If you want to send more datas, you can add a field in this structure.

## Communication flow

Every 100 µs in `loop_critical_task()`:

1. MASTER sends a frame with:
   - `voltage_ref = 32.0F`
   - `status_code = POWER`
2. FOLLOWER receives in `reception_function()`, updates mode, fills `voltage_meas`, and replies immediately.
3. MASTER receives the reply and stores `voltage_meas`.


## Serial console

Available keys:

- `h`: help menu
- `i`: IDLE mode
- `p`: POWER mode

press p in the serial monitor to start the transmission.

## Task architecture

The example has the following three tasks :

- `loop_background_task` (status print + LED + 2 s sleep)
- `loop_critical_task` (100 µs periodic control + RS485 transmission)
- `loop_communication_task` (serial menu)

RS485 is configured with the following function, setting the speed of the communication to 20Mbit/s. 

```cpp
communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx), reception_function, SPEED_20M);
```

## Customization

To send more data:

1. Add a field in `RS485_frame`
2. Fill it before `memcpy(buffer_tx, &frame_tx, sizeof(frame_tx));`
3. Read it after `memcpy(&frame_rx, buffer_rx, sizeof(frame_rx));`