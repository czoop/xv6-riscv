//
// driver for qemu's virtio network device.
// uses qemu's mmio interface to virtio.
// qemu presents a "legacy" virtio interface.
//
// qemu ... -device virtio-net-device,netdev=net0,bus=virtio-mmio-bus.1
//
//

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"
#include "virtio.h"

// the address of virtio mmio register r.
#define R(r) ((volatile uint32 *)(VIRTIO1 + (r)))

static struct network
{
  // the virtio driver and device mostly communicate through a set of
  // structures in RAM. pages[] allocates that memory. pages[] is a
  // global (instead of calls to kalloc()) because it must consist of
  // two contiguous pages of page-aligned physical memory.
  char pages[2 * PGSIZE];

  // pages[] is divided into three regions (descriptors, avail, and
  // used), as explained in Section 2.6 of the virtio specification
  // for the legacy interface.
  // https://docs.oasis-open.org/virtio/virtio/v1.1/virtio-v1.1.pdf

  // the first region of pages[] is a set (not a ring) of DMA
  // descriptors, with which the driver tells the device where to read
  // and write individual network operations. there are NUM descriptors.
  // most commands consist of a "chain" (a linked list) of a couple of
  // these descriptors.
  // points into pages[].
  struct virtq_desc *desc;

  // next is a ring in which the driver writes descriptor numbers
  // that the driver would like the device to process.  it only
  // includes the head descriptor of each chain. the ring has
  // NUM elements.
  // points into pages[].
  struct virtq_avail *avail;

  // finally a ring in which the device writes descriptor numbers that
  // the device has finished processing (just the head of each chain).
  // there are NUM used ring entries.
  // points into pages[].
  struct virtq_used *used;

  // our own book-keeping.
  char free[NUM];  // is a descriptor free?
  uint16 used_idx; // we've looked this far in used[2..NUM].

  // track info about in-flight operations,
  // for use when completion interrupt arrives.
  // indexed by first descriptor index of chain.
  struct
  {
    struct buf *b;
    char status;
  } info[NUM];

  // network command headers.
  // one-for-one with descriptors, for convenience.
  struct virtio_blk_req ops[NUM];

  struct spinlock vnetwork_lock;

} __attribute__((aligned(PGSIZE))) network[2];

void read_virtio_status()
{
  // https://docs.oasis-open.org/virtio/virtio/v1.1/cs01/virtio-v1.1-cs01.html#x1-156001r2
  // Interrupt Status 0x060
  // Device Status 0x070

  printf("VIRTIO INTERRUPT STATUS: %p\n", (char)*R(VIRTIO_MMIO_INTERRUPT_STATUS));
}

void virtio_network_init(void)
{
  uint32 status = 0;

  initlock(&network[0].vnetwork_lock, "virtio_network");
  initlock(&network[1].vnetwork_lock, "virtio_network");

  if (*R(VIRTIO_MMIO_MAGIC_VALUE) != 0x74726976 ||
      *R(VIRTIO_MMIO_VERSION) != 1 ||
      *R(VIRTIO_MMIO_DEVICE_ID) != 1 ||
      *R(VIRTIO_MMIO_VENDOR_ID) != 0x554d4551)
  {
    panic("could not find virtio network");
  }

  status |= VIRTIO_CONFIG_S_ACKNOWLEDGE;
  *R(VIRTIO_MMIO_STATUS) = status;
  status |= VIRTIO_CONFIG_S_DRIVER;
  *R(VIRTIO_MMIO_STATUS) = status;

  // negotiate features

  uint64 features = *R(VIRTIO_MMIO_DEVICE_FEATURES);
  features &= ~(1 << VIRTIO_NET_F_MAC);
  features &= ~(1 << VIRTIO_NET_F_STATUS);
  features &= ~(1 << VIRTIO_F_ANY_LAYOUT);
  features &= ~(1 << VIRTIO_RING_F_EVENT_IDX);
  features &= ~(1 << VIRTIO_RING_F_INDIRECT_DESC);
  *R(VIRTIO_MMIO_DRIVER_FEATURES) = features;

  // tell device that feature negotiation is complete.
  status |= VIRTIO_CONFIG_S_FEATURES_OK;
  *R(VIRTIO_MMIO_STATUS) = status;

  // tell device we're completely ready.
  status |= VIRTIO_CONFIG_S_DRIVER_OK;
  *R(VIRTIO_MMIO_STATUS) = status;

  *R(VIRTIO_MMIO_GUEST_PAGE_SIZE) = PGSIZE;

  // initialize queue 0.
  *R(VIRTIO_MMIO_QUEUE_SEL) = 0;
  uint32 rq_max = *R(VIRTIO_MMIO_QUEUE_NUM_MAX);
  if (rq_max == 0)
    panic("virtio network has no queue 0");
  if (rq_max < NUM)
    panic("virtio network max queue too short");
  *R(VIRTIO_MMIO_QUEUE_NUM) = NUM;
  memset(network[0].pages, 0, sizeof(network[0].pages));
  *R(VIRTIO_MMIO_QUEUE_PFN) = ((uint64)network[0].pages) >> PGSHIFT;

  // initialize queue 1.
  *R(VIRTIO_MMIO_QUEUE_SEL) = 1;
  uint32 tq_max = *R(VIRTIO_MMIO_QUEUE_NUM_MAX);
  if (tq_max == 0)
    panic("virtio network has no queue 1");
  if (tq_max < NUM)
    panic("virtio network max queue too short");
  *R(VIRTIO_MMIO_QUEUE_NUM) = NUM;
  memset(network[1].pages, 0, sizeof(network[1].pages));
  *R(VIRTIO_MMIO_QUEUE_PFN) = ((uint64)network[1].pages) >> PGSHIFT;

  // desc = pages -- num * virtq_desc
  // avail = pages + 0x40 -- 2 * uint16, then num * uint16
  // used = pages + 4096 -- 2 * uint16, then num * vRingUsedElem

  network[0].desc = (struct virtq_desc *)network[0].pages;
  network[0].avail = (struct virtq_avail *)(network[0].pages + NUM * sizeof(struct virtq_desc));
  network[0].used = (struct virtq_used *)(network[0].pages + PGSIZE);

  network[1].desc = (struct virtq_desc *)network[1].pages;
  network[1].avail = (struct virtq_avail *)(network[1].pages + NUM * sizeof(struct virtq_desc));
  network[1].used = (struct virtq_used *)(network[1].pages + PGSIZE);

  // all NUM descriptors start out unused.
  for (int i = 0; i < NUM; i++)
  {
    network[0].free[i] = 1;
    network[1].free[i] = 1;
  }

  // plic.c and trap.c arrange for interrupts from VIRTIO0_IRQ.
  printf("Finished VirtIO Init\n");

  struct virtio_net_config * nc = (struct virtio_net_config*) R(0x100);
  struct network_buf nb;
  nb.hdr.flags = 0;
  nb.hdr.gso_type = VIRTIO_NET_HDR_GSO_NONE;
  nb.hdr.hdr_length = sizeof(struct virtio_net_hdr);
  nb.hdr.gso_size = 1514;
  nb.hdr.csum_start = 8;
  nb.hdr.csum_offset = 1510;


  printf("%x:%x:0%x:%x:%x:%x\n", nc->mac[0], nc->mac[1], nc->mac[2], nc->mac[3], nc->mac[4], nc->mac[5]);


  virtio_network_rw(&nb, 0);
}

// find a free descriptor, mark it non-free, return its index.
static int
alloc_desc(int q)
{
  for (int i = 0; i < NUM; i++) {
    if (network[q].free[i]) {
      network[q].free[i] = 0;
      return i;
    }
  }
  return -1;
}

// mark a descriptor as free.
static void
free_desc(int i, int q)
{
  if (i >= NUM)
    panic("free_desc 1");
  if (network[q].free[i])
    panic("free_desc 2");
  network[q].desc[i].addr = 0;
  network[q].desc[i].len = 0;
  network[q].desc[i].flags = 0;
  network[q].desc[i].next = 0;
  network[q].free[i] = 1;
  wakeup(&network[q].free[0]);
}

// free a chain of descriptors.
static void
free_chain(int i, int q)
{
  while (1)
  {
    int flag = network[q].desc[i].flags;
    int nxt = network[q].desc[i].next;
    free_desc(i, q);
    if (flag & VRING_DESC_F_NEXT)
      i = nxt;
    else
      break;
  }
}

// allocate three descriptors (they need not be contiguous).
// network transfers always use three descriptors.
static int
alloc3_desc(int *idx, int q)
{
  for (int i = 0; i < 3; i++)
  {
    idx[i] = alloc_desc(q);
    if (idx[i] < 0)
    {
      for (int j = 0; j < i; j++)
        free_desc(idx[j], q);
      return -1;
    }
  }
  return 0;
}

void virtio_network_rw(struct network_buf *b, int write)
{
  // Assuming write == 1
  printf("Writing packet!\n");

  acquire(&network[write].vnetwork_lock);

  int idx[3];
  while (1) {
    if (alloc3_desc(idx, write) == 0) {
      break;
    }
    sleep(&network[write].free[0], &network[write].vnetwork_lock);
  }

  // Descriptors in idx array acquired
  network[write].desc->addr = (uint64) &b;


  release(&network[write].vnetwork_lock);
}

void virtio_network_intr()
{
  printf("virtio_network_intr received\n");
  /* acquire(&network.vnetwork_lock); */

  /* // the device won't raise another interrupt until we tell it */
  /* // we've seen this interrupt, which the following line does. */
  /* // this may race with the device writing new entries to */
  /* // the "used" ring, in which case we may process the new */
  /* // completion entries in this interrupt, and have nothing to do */
  /* // in the next interrupt, which is harmless. */
  /* *R(VIRTIO_MMIO_INTERRUPT_ACK) = *R(VIRTIO_MMIO_INTERRUPT_STATUS) & 0x3; */

  /* __sync_synchronize(); */

  /* // the device increments network.used->idx when it */
  /* // adds an entry to the used ring. */

  /* while (network.used_idx != network.used->idx) */
  /* { */
  /*   __sync_synchronize(); */
  /*   int id = network.used->ring[network.used_idx % NUM].id; */

  /*   if (network.info[id].status != 0) */
  /*     panic("virtio_network_intr status"); */

  /*   struct buf *b = network.info[id].b; */
  /*   b->disk = 0; // network is done with buf */
  /*   wakeup(b); */

  /*   network.used_idx += 1; */
  /* } */

  /* release(&network.vnetwork_lock); */
}
