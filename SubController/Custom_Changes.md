# Custom_Changes After Code Regeneration

## DMA_NORMAL To DMA_CIRCULAR

## DMA_FIFOMODE_DISABLE

## STM32H750VBTx_FLASH.ld

```c
.dma (NOLOAD) : ALIGN(4) {
        KEEP(*(.dma))
  } >RAM_D1
```
