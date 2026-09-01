# STM32 network interface

This directory contains the unified FreeRTOS+TCP Ethernet network interface
for STM32F1, STM32F2, STM32F4, STM32F7, STM32H5, STM32H7, STM32H7RS, and
STM32N6 devices.

The network interface contains the portable Ethernet logic. The application
must still provide board-specific clock, GPIO, interrupt, linker, and memory
configuration.

## Configuration

Enable the Ethernet HAL module. The network interface also requires zero-copy
receive and transmit buffers:

```c
#define HAL_ETH_MODULE_ENABLED

#define ipconfigZERO_COPY_RX_DRIVER  1
#define ipconfigZERO_COPY_TX_DRIVER  1
```

The following options are recommended so the hardware can perform filtering,
checksum offload, and batched receive-event delivery:

```c
#define ipconfigUSE_LINKED_RX_MESSAGES              1
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES  1
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM       1
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM       1
```

The driver uses file-static HAL, PHY, task, semaphore, and address-filter
state. Consequently, one Ethernet peripheral instance is supported.

## HAL Ethernet initialization

Provide `HAL_ETH_MspInit()` and, when deinitialization is required,
`HAL_ETH_MspDeInit()` in the application. A CubeMX-generated implementation is
the best starting point because the exact clocks, alternate functions, pinout,
and interrupt name depend on the STM32 family, part, package, and board.

The initialization must:

1. Enable all Ethernet MAC, transmit, receive, and peripheral clocks required
   by the selected family.
2. Enable the GPIO port clocks and configure every MII or RMII signal used by
   the board.
3. Configure and enable the Ethernet interrupt at a priority from which
   FreeRTOS APIs may be called. Use the priority-number representation expected
   by the STM32 HAL/CMSIS call rather than a pre-shifted register value.

The deinitialization should disable the interrupt, release the GPIOs, and
disable the clocks enabled during initialization.

The generic structure is:

```c
void HAL_ETH_MspInit( ETH_HandleTypeDef * pxEthHandle )
{
    if( pxEthHandle->Instance == ETH_INSTANCE_FOR_THIS_DEVICE )
    {
        /* Enable the family-specific Ethernet and GPIO clocks. */
        /* Configure the board-specific MII or RMII pins. */
        /* Configure and enable the Ethernet interrupt. */
    }
}

void HAL_ETH_MspDeInit( ETH_HandleTypeDef * pxEthHandle )
{
    if( pxEthHandle->Instance == ETH_INSTANCE_FOR_THIS_DEVICE )
    {
        /* Disable the Ethernet interrupt. */
        /* Deinitialize the board-specific Ethernet pins. */
        /* Disable the clocks enabled by HAL_ETH_MspInit(). */
    }
}
```

Replace `ETH_INSTANCE_FOR_THIS_DEVICE` with the instance exposed by the device
HAL, such as `ETH` or `ETH1`.

## DMA memory and cache coherency

Place the following sections in DMA-accessible memory:

- `.TxDescripSection`
- `.RxDescripSection`
- `.EthBuffersSection`

The descriptor sections must be non-cacheable. Packet buffers may also be
non-cacheable, or the application may define `niEMAC_USE_MPU` as
`ipconfigDISABLE` so that the network interface performs explicit data-cache
maintenance.

If an MPU is used, configure its regions in application startup code using the
actual linker-provided section boundaries and sizes. Do not copy fixed region
sizes from another STM32 part or linker layout.
