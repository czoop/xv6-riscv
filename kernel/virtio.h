//
// virtio device definitions.
// for both the mmio interface, and virtio descriptors.
// only tested with qemu.
// this is the "legacy" virtio interface.
//
// the virtio spec:
// https://docs.oasis-open.org/virtio/virtio/v1.1/virtio-v1.1.pdf
//

// virtio mmio control registers, mapped starting at 0x10001000.
// from qemu virtio_mmio.h
#define VIRTIO_MMIO_MAGIC_VALUE		0x000 // 0x74726976
#define VIRTIO_MMIO_VERSION		0x004 // version; 1 is legacy
#define VIRTIO_MMIO_DEVICE_ID		0x008 // device type; 1 is net, 2 is disk
#define VIRTIO_MMIO_VENDOR_ID		0x00c // 0x554d4551
#define VIRTIO_MMIO_DEVICE_FEATURES	0x010
#define VIRTIO_MMIO_DRIVER_FEATURES	0x020
#define VIRTIO_MMIO_GUEST_PAGE_SIZE	0x028 // page size for PFN, write-only
#define VIRTIO_MMIO_QUEUE_SEL		0x030 // select queue, write-only
#define VIRTIO_MMIO_QUEUE_NUM_MAX	0x034 // max size of current queue, read-only
#define VIRTIO_MMIO_QUEUE_NUM		0x038 // size of current queue, write-only
#define VIRTIO_MMIO_QUEUE_ALIGN		0x03c // used ring alignment, write-only
#define VIRTIO_MMIO_QUEUE_PFN		0x040 // physical page number for queue, read/write
#define VIRTIO_MMIO_QUEUE_READY		0x044 // ready bit
#define VIRTIO_MMIO_QUEUE_NOTIFY	0x050 // write-only
#define VIRTIO_MMIO_INTERRUPT_STATUS	0x060 // read-only
#define VIRTIO_MMIO_INTERRUPT_ACK	0x064 // write-only
#define VIRTIO_MMIO_STATUS		0x070 // read/write

// status register bits, from qemu virtio_config.h
#define VIRTIO_CONFIG_S_ACKNOWLEDGE	1
#define VIRTIO_CONFIG_S_DRIVER		2
#define VIRTIO_CONFIG_S_DRIVER_OK	4
#define VIRTIO_CONFIG_S_FEATURES_OK	8

// block device feature bits
#define VIRTIO_BLK_F_RO              5	/* Disk is read-only */
#define VIRTIO_BLK_F_SCSI            7	/* Supports scsi command passthru */
#define VIRTIO_BLK_F_CONFIG_WCE     11	/* Writeback mode available in config */
#define VIRTIO_BLK_F_MQ             12	/* support more than one vq */

// network device feature bits
#define VIRTIO_NET_F_MAC            5
#define VIRTIO_NET_F_STATUS         16

// network device header flags
#define VIRTIO_NET_HDR_F_NEEDS_CSUM 1

#define VIRTIO_NET_HDR_GSO_NONE     0

// device feature bits
#define VIRTIO_F_ANY_LAYOUT         27
#define VIRTIO_RING_F_INDIRECT_DESC 28
#define VIRTIO_RING_F_EVENT_IDX     29

// this many virtio descriptors.
// must be a power of two.
#define NUM 8

// a single descriptor, from the spec.
struct virtq_desc {
  uint64 addr;
  uint32 len;
  uint16 flags;
  uint16 next;
};
#define VRING_DESC_F_NEXT  1 // chained with another descriptor
#define VRING_DESC_F_WRITE 2 // device writes (vs read)

// the (entire) avail ring, from the spec.
struct virtq_avail {
  uint16 flags; // always zero
  uint16 idx;   // driver will write ring[idx] next
  uint16 ring[NUM]; // descriptor numbers of chain heads
  uint16 unused;
};

// one entry in the "used" ring, with which the
// device tells the driver about completed requests.
struct virtq_used_elem {
  uint32 id;   // index of start of completed descriptor chain
  uint32 len;
};

struct virtq_used {
  uint16 flags; // always zero
  uint16 idx;   // device increments when it adds a ring[] entry
  struct virtq_used_elem ring[NUM];
};

// these are specific to virtio block devices, e.g. disks,
// described in Section 5.2 of the spec.

#define VIRTIO_BLK_T_IN  0 // read the disk
#define VIRTIO_BLK_T_OUT 1 // write the disk

// the format of the first descriptor in a disk request.
// to be followed by two more descriptors containing
// the block, and a one-byte status.
struct virtio_blk_req {
  uint32 type; // VIRTIO_BLK_T_IN or ..._OUT
  uint32 reserved;
  uint64 sector;
};

struct virtio_net_config {
    uint8 mac[6];
    uint16 status;
    uint16 max_virtqueue_pairs;
    uint16 mtu;
};

struct virtio_net_hdr {
  uint8 flags;                // Bit 0: Needs checksum; Bit 1: Received packet has valid data;
                                // Bit 2: If VIRTIO_NET_F_RSC_EXT was negotiated, the device processes
                                // duplicated ACK segments, reports number of coalesced TCP segments in ChecksumStart
                                // field and number of duplicated ACK segments in ChecksumOffset field,
                                // and sets bit 2 in Flags(VIRTIO_NET_HDR_F_RSC_INFO)
  uint8  gso_type;  // 0:None 1:TCPv4 3:UDP 4:TCPv6 0x80:ECN
  uint16 hdr_length;        // Size of header to be used during segmentation.
  uint16 gso_size;       // Maximum segment size (not including header).
  uint16 csum_start;       // The position to begin calculating the checksum.
  uint16 csum_offset;      // The position after ChecksumStart to store the checksum.
  uint16 num_buffers;
};


struct eth_frame
{
    uchar pre[7];           // not needed anymore?
    uchar sfd;              // always set to 0b10101011
    uchar dest[6];          // 6B mac addr
    uchar src[6];           // 6B mac addr
    uchar len[2];           // length of the entire ethernet frame, cannot be larger than 1500
    uchar data[1488];       // max = 1500B, min = 46B; pad w/ 0s otherwise
    uchar crc[4];           // 32 bit hash (csum) over dest, src, len, and data
};

// have to put it here for some damn reason
struct network_buf
{
    struct virtio_net_hdr hdr;
    struct eth_frame data;
    /* uchar data[1514]; */
};
