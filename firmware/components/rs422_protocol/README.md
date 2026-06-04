# `rs422_protocol` — placeholder

Phase 5 work. Planned content:

- UART driver wrap with DMA + ring buffer
- Framed protocol (STX/LEN/CMD/SEQ/PAYLOAD/CRC16/ETX) or CSP/StackFlow
- Command dispatcher (CAPTURE_TO_OBC, CAPTURE_TO_SSTV, PING, ABORT)
- File-streaming mode for the JPEG payload

Until then: empty register; `firmware/main/main.c` does not REQUIRE this
yet, so it's optional during Phase 1.
