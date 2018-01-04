// W5100 registers
#define  W5100_MR                 0x0000      /* Mode Register */
#define  W5100_GAR                0x0001      /* Gateway Address: 0x0001 to 0x0004 */
#define  W5100_SUBR               0x0005      /* Subnet mask Address: 0x0005 to 0x0008 */
#define  W5100_SHAR               0x0009      /* Source Hardware Address (MAC): 0x0009 to 0x000E */
#define  W5100_SIPR               0x000F      /* Source IP Address: 0x000F to 0x0012 */
#define  W5100_IMR                0x0016      /* Interrupt Mask Register */
#define  W5100_SKT_REG_BASE       0x0400      /* start of socket registers */
#define  W5100_SKT_OFFSET         0x0100      /* offset to each socket regester set */
#define  W5100_SKT_BASE(n)        (W5100_SKT_REG_BASE+(n*W5100_SKT_OFFSET))

// socket register offsets
#define  W5100_MR_OFFSET          0x0000      /* socket Mode Register offset */
#define  W5100_CR_OFFSET          0x0001      /* socket Command Register offset */
#define  W5100_IR_OFFSET          0x0002      /* socket Interrupt Register offset */
#define  W5100_SR_OFFSET          0x0003      /* socket Status Register offset */
#define  W5100_PORT_OFFSET        0x0004      /* socket Port Register offset (2 bytes) */
#define  W5100_DHAR_OFFSET        0x0006      /* socket Destination Hardware Address Register (MAC, 6 bytes) */
#define  W5100_DIPR_OFFSET        0x000C      /* socket Destination IP Address Register (IP, 4 bytes) */
#define  W5100_DPORT_OFFSET       0x0010      /* socket Destination Port Register (2 bytes) */
#define  W5100_MSS_OFFSET         0x0012      /* socket Maximum Segment Size (2 bytes) */
#define  W5100_PROTO_OFFSET       0x0014      /* socket IP Protocol Register */
#define  W5100_TOS_OFFSET         0x0015      /* socket Type Of Service Register */
#define  W5100_TTL_OFFSET         0x0016      /* socket Time To Live Register */
#define  W5100_TX_FSR_OFFSET      0x0020      /* socket Transmit Free Size Register (2 bytes) */
#define  W5100_TX_RR_OFFSET       0x0022      /* socket Transmit Read Pointer Register (2 bytes) */
#define  W5100_TX_WR_OFFSET       0x0024      /* socket Transmit Write Pointer Register (2 bytes) */
#define  W5100_RX_RSR_OFFSET      0x0026      /* socket Receive Received Size Register (2 bytes) */
#define  W5100_RX_RD_OFFSET       0x0028      /* socket Receive Read Pointer Register (2 bytes) */

// Device Mode Register
#define  W5100_MR_SOFTRST         (1<<7)      /* soft-reset */
#define  W5100_MR_PINGBLK         (1<<4)      /* block responses to ping request */
#define  W5100_MR_PPPOE           (1<<3)      /* enable PPPoE */
#define  W5100_MR_AUTOINC         (1<<1)      /* address autoincrement (indirect interface ONLY!) */
#define  W5100_MR_INDINT          (1<<0)      /* use indirect interface (parallel interface ONLY!) */

// Socket mode register
#define  W5100_SKT_MR_CLOSE       0x00        /* Unused socket */
#define  W5100_SKT_MR_TCP         0x01        /* TCP */
#define  W5100_SKT_MR_UDP         0x02        /* UDP */
#define  W5100_SKT_MR_IPRAW       0x03        /* IP LAYER RAW SOCK */
#define  W5100_SKT_MR_MACRAW      0x04        /* MAC LAYER RAW SOCK */
#define  W5100_SKT_MR_PPPOE       0x05        /* PPPoE */
#define  W5100_SKT_MR_ND          0x20        /* No Delayed Ack(TCP) flag */
#define  W5100_SKT_MR_MULTI       0x80        /* support multicasting */

// Socket command register
#define  W5100_SKT_CR_OPEN        0x01        /* open the socket */
#define  W5100_SKT_CR_LISTEN      0x02        /* wait for TCP connection (server mode) */
#define  W5100_SKT_CR_CONNECT     0x04        /* listen for TCP connection (client mode) */
#define  W5100_SKT_CR_DISCON      0x08        /* close TCP connection */
#define  W5100_SKT_CR_CLOSE       0x10        /* mark socket as closed (does not close TCP connection) */
#define  W5100_SKT_CR_SEND        0x20        /* transmit data in TX buffer */
#define  W5100_SKT_CR_SEND_MAC    0x21        /* SEND, but uses destination MAC address (UDP only) */
#define  W5100_SKT_CR_SEND_KEEP   0x22        /* SEND, but sends 1-byte packet for keep-alive (TCP only) */
#define  W5100_SKT_CR_RECV        0x40        /* receive data into RX buffer */

// Socket status register
#define  W5100_SKT_SR_CLOSED      0x00        /* closed */
#define  W5100_SKT_SR_INIT        0x13        /* init state */
#define  W5100_SKT_SR_LISTEN      0x14        /* listen state */
#define  W5100_SKT_SR_SYNSENT     0x15        /* connection state */
#define  W5100_SKT_SR_SYNRECV     0x16        /* connection state */
#define  W5100_SKT_SR_ESTABLISHED 0x17        /* success to connect */
#define  W5100_SKT_SR_FIN_WAIT    0x18        /* closing state */
#define  W5100_SKT_SR_CLOSING     0x1A        /* closing state */
#define  W5100_SKT_SR_TIME_WAIT   0x1B        /* closing state */
#define  W5100_SKT_SR_CLOSE_WAIT  0x1C        /* closing state */
#define  W5100_SKT_SR_LAST_ACK    0x1D        /* closing state */
#define  W5100_SKT_SR_UDP         0x22        /* UDP socket */
#define  W5100_SKT_SR_IPRAW       0x32        /* IP raw mode socket */
#define  W5100_SKT_SR_MACRAW      0x42        /* MAC raw mode socket */
#define  W5100_SKT_SR_PPPOE       0x5F        /* PPPOE socket */

// TX and RX buffers
#define  W5100_TXBUFADDR          0x4000      /* W5100 Send Buffer Base Address */
#define  W5100_RXBUFADDR          0x6000      /* W5100 Read Buffer Base Address */
#define  W5100_BUFSIZE            0x0800      /* W5100 buffers are sized 2K */
#define  W5100_TX_BUF_MASK        0x07FF      /* Tx 2K Buffer Mask */
#define  W5100_RX_BUF_MASK        0x07FF      /* Rx 2K Buffer Mask */

#define W5100_WRITE_OPCODE        0xF0
#define W5100_READ_OPCODE         0x0F
