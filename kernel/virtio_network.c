//
// driver for qemu's virtio network device.
// uses qemu's mmio interface to virtio.
// qemu presents a "legacy" virtio interface.
//
// qemu ... -drive file=fs.img,if=none,format=raw,id=x0 -device virtio-blk-device,drive=x0,bus=virtio-mmio-bus.0
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

} __attribute__((aligned(PGSIZE))) network;

void virtio_network_init(void)
{
  uint32 status = 0;

  initlock(&network.vnetwork_lock, "virtio_network");

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
  /*
  uint64 features = *R(VIRTIO_MMIO_DEVICE_FEATURES);
  features &= ~(1 << VIRTIO_BLK_F_RO);
  features &= ~(1 << VIRTIO_BLK_F_SCSI);
  features &= ~(1 << VIRTIO_BLK_F_CONFIG_WCE);
  features &= ~(1 << VIRTIO_BLK_F_MQ);
  features &= ~(1 << VIRTIO_F_ANY_LAYOUT);
  features &= ~(1 << VIRTIO_RING_F_EVENT_IDX);
  features &= ~(1 << VIRTIO_RING_F_INDIRECT_DESC);
  *R(VIRTIO_MMIO_DRIVER_FEATURES) = features;
  */

  // tell device that feature negotiation is complete.
  /* status |= VIRTIO_CONFIG_S_FEATURES_OK; */
  /* *R(VIRTIO_MMIO_STATUS) = status; */

  // tell device we're completely ready.
  status |= VIRTIO_CONFIG_S_DRIVER_OK;
  *R(VIRTIO_MMIO_STATUS) = status;

  /* *R(VIRTIO_MMIO_GUEST_PAGE_SIZE) = PGSIZE; */

  // initialize queue 0.
  *R(VIRTIO_MMIO_QUEUE_SEL) = 0;
  uint32 max = *R(VIRTIO_MMIO_QUEUE_NUM_MAX);
  if (max == 0)
    panic("virtio network has no queue 0");
  if (max < NUM)
    panic("virtio network max queue too short");
  *R(VIRTIO_MMIO_QUEUE_NUM) = NUM;
  memset(network.pages, 0, sizeof(network.pages));
  *R(VIRTIO_MMIO_QUEUE_PFN) = ((uint64)network.pages) >> PGSHIFT;

  // desc = pages -- num * virtq_desc
  // avail = pages + 0x40 -- 2 * uint16, then num * uint16
  // used = pages + 4096 -- 2 * uint16, then num * vRingUsedElem

  network.desc = (struct virtq_desc *)network.pages;
  network.avail = (struct virtq_avail *)(network.pages + NUM * sizeof(struct virtq_desc));
  network.used = (struct virtq_used *)(network.pages + PGSIZE);

  // all NUM descriptors start out unused.
  for (int i = 0; i < NUM; i++)
    network.free[i] = 1;

  // plic.c and trap.c arrange for interrupts from VIRTIO0_IRQ.
}

// find a free descriptor, mark it non-free, return its index.
static int
alloc_desc()
{
  for (int i = 0; i < NUM; i++)
  {
    if (network.free[i])
    {
      network.free[i] = 0;
      return i;
    }
  }
  return -1;
}

// mark a descriptor as free.
static void
free_desc(int i)
{
  if (i >= NUM)
    panic("free_desc 1");
  if (network.free[i])
    panic("free_desc 2");
  network.desc[i].addr = 0;
  network.desc[i].len = 0;
  network.desc[i].flags = 0;
  network.desc[i].next = 0;
  network.free[i] = 1;
  wakeup(&network.free[0]);
}

// free a chain of descriptors.
static void
free_chain(int i)
{
  while (1)
  {
    int flag = network.desc[i].flags;
    int nxt = network.desc[i].next;
    free_desc(i);
    if (flag & VRING_DESC_F_NEXT)
      i = nxt;
    else
      break;
  }
}

// allocate three descriptors (they need not be contiguous).
// network transfers always use three descriptors.
static int
alloc3_desc(int *idx)
{
  for (int i = 0; i < 3; i++)
  {
    idx[i] = alloc_desc();
    if (idx[i] < 0)
    {
      for (int j = 0; j < i; j++)
        free_desc(idx[j]);
      return -1;
    }
  }
  return 0;
}

void virtio_network_rw(struct buf *b, int write)
{
  uint64 sector = b->blockno * (BSIZE / 512);

  acquire(&network.vnetwork_lock);

  // the spec's Section 5.2 says that legacy block operations use
  // three descriptors: one for type/reserved/sector, one for the
  // data, one for a 1-byte status result.

  // allocate the three descriptors.
  int idx[3];
  while (1)
  {
    if (alloc3_desc(idx) == 0)
    {
      break;
    }
    sleep(&network.free[0], &network.vnetwork_lock);
  }

  // format the three descriptors.
  // qemu's virtio-blk.c reads them.

  struct virtio_blk_req *buf0 = &network.ops[idx[0]];

  if (write)
    buf0->type = VIRTIO_BLK_T_OUT; // write the network
  else
    buf0->type = VIRTIO_BLK_T_IN; // read the network
  buf0->reserved = 0;
  buf0->sector = sector;

  network.desc[idx[0]].addr = (uint64)buf0;
  network.desc[idx[0]].len = sizeof(struct virtio_blk_req);
  network.desc[idx[0]].flags = VRING_DESC_F_NEXT;
  network.desc[idx[0]].next = idx[1];

  network.desc[idx[1]].addr = (uint64)b->data;
  network.desc[idx[1]].len = BSIZE;
  if (write)
    network.desc[idx[1]].flags = 0; // device reads b->data
  else
    network.desc[idx[1]].flags = VRING_DESC_F_WRITE; // device writes b->data
  network.desc[idx[1]].flags |= VRING_DESC_F_NEXT;
  network.desc[idx[1]].next = idx[2];

  network.info[idx[0]].status = 0xff; // device writes 0 on success
  network.desc[idx[2]].addr = (uint64)&network.info[idx[0]].status;
  network.desc[idx[2]].len = 1;
  network.desc[idx[2]].flags = VRING_DESC_F_WRITE; // device writes the status
  network.desc[idx[2]].next = 0;

  // record struct buf for virtio_network_intr().
  b->disk = 1;
  network.info[idx[0]].b = b;

  // tell the device the first index in our chain of descriptors.
  network.avail->ring[network.avail->idx % NUM] = idx[0];

  __sync_synchronize();

  // tell the device another avail ring entry is available.
  network.avail->idx += 1; // not % NUM ...

  __sync_synchronize();

  *R(VIRTIO_MMIO_QUEUE_NOTIFY) = 0; // value is queue number

  // Wait for virtio_network_intr() to say request has finished.
  while (b->disk == 1)
  {
    sleep(b, &network.vnetwork_lock);
  }

  network.info[idx[0]].b = 0;
  free_chain(idx[0]);

  release(&network.vnetwork_lock);
}

void virtio_network_intr()
{
  acquire(&network.vnetwork_lock);

  // the device won't raise another interrupt until we tell it
  // we've seen this interrupt, which the following line does.
  // this may race with the device writing new entries to
  // the "used" ring, in which case we may process the new
  // completion entries in this interrupt, and have nothing to do
  // in the next interrupt, which is harmless.
  *R(VIRTIO_MMIO_INTERRUPT_ACK) = *R(VIRTIO_MMIO_INTERRUPT_STATUS) & 0x3;

  __sync_synchronize();

  // the device increments network.used->idx when it
  // adds an entry to the used ring.

  while (network.used_idx != network.used->idx)
  {
    __sync_synchronize();
    int id = network.used->ring[network.used_idx % NUM].id;

    if (network.info[id].status != 0)
      panic("virtio_network_intr status");

    struct buf *b = network.info[id].b;
    b->disk = 0; // network is done with buf
    wakeup(b);

    network.used_idx += 1;
  }

  release(&network.vnetwork_lock);
}
