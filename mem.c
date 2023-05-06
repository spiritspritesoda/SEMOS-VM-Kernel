#include "types.h"
#include "memlayout.h"
#include "elf.h"
#include "riscv.h"
#include "mem.h"
#include "string.h"
#include "console.h"


///////////////////////////////////////////////////////////////////////////////
// Static Helper Prototypes
///////////////////////////////////////////////////////////////////////////////
static pagetable_t make_kernel_pagetable(void);
static pte_t * walk_pgtable(pagetable_t pagetable, uint64 va, int alloc);
static void kernel_map_pages(pagetable_t, uint64, uint64, uint64, int);
static int  kernel_map_range(pagetable_t, uint64, uint64, uint64, int);
static void free_range(void *, void *);



///////////////////////////////////////////////////////////////////////////////
// Global Definitions
///////////////////////////////////////////////////////////////////////////////
extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct frame {
  struct frame* next;
};

struct frame *frame_table;

/*
 * the kernel's page table.
 */
pagetable_t kernel_pagetable;

extern char etext[];  // kernel.ld sets this to end of kernel code.

extern char trampoline[]; // trampoline.S


///////////////////////////////////////////////////////////////////////////////
// Page Allocation and Virtual Memory API
///////////////////////////////////////////////////////////////////////////////

// Initialize Virtual memory and activate paging.
void
vm_init(void)
{
  free_range(end, (void*)PHYSTOP);
  kernel_pagetable = make_kernel_pagetable();
  w_satp(MAKE_SATP(kernel_pagetable));
  sfence_vma();
}


// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *
struct frame* vm_page_alloc(void) {
  struct frame* new_page = frame_table;

  // check if a frame is available
  if (new_page == NULL) {
    return NULL;
  }

  // advance the frame_table pointer to the next available frame
  frame_table = frame_table->next;

  // initialize the new page with zeros
  memset(new_page->physical_addr, 0, PAGE_SIZE);
  // return the first frame from the frame_table linked list
  return new_page;
}



void vm_page_free(void* v) {
  uint32_t index;
 // Calculate the index of the physical page to free
  index = ((uintptr_t)v - mem_start) / PAGE_SIZE;
  // Sanity check: make sure the index is valid
  if (index >= mem_size / PAGE_SIZE) {
    panic("vm_page_free: invalid index");
  }
  // Mark the page as free in the page table
  page_table[index] = PAGE_FREE;
  // Link the page back into the frame table as the first free frame
  frame_table[index].next = first_free_frame;
  first_free_frame = index;
}


pagetable_t
*vm_create_pagetable(void) {
    // Allocate a page frame to store the page directory
    uint32_t page_frame_index = vm_page_alloc();
    if (page_frame_index == 0) {
        return 0; // Out of memory
    }

    // Clear the contents of the page frame (i.e., set all PTEs to invalid)
    pde_t *page_dir = (pde_t *) vm_index_to_vaddr(page_frame_index);
    memset(page_dir, 0, PAGE_SIZE);

    // Return a pointer to the new page directory
    return page_dir;
}



// Look up a virtual address, return the physical address,
// or 0 if not mapped.
uint64
vm_lookup(pagetable_t pagetable, uint64 va)
// Look up the PTE for the virtual address
pte_t* pte = walk_pgtable(pgdir, vaddr, /*create*/ false);
// If the PTE is invalid, return 0
if (pte == 0 || (*pte & PTE_PRESENT) == 0) {
  return 0;
}
// Convert the PTE to a physical address and return it
return ((*pte & PTE_FRAME) | (vaddr & PAGE_OFFSET));
}


// Insert a page into the page table which maps the page containing
// the virtual address va onto the page frame at the physical address
// pa. va can be any arbitrary address, but pa must be aligned to
// a physical page. Returns 0 on success, -1 on failure.
int vm_page_insert(pde_t* pgdir, uintptr_t va, physaddr_t pa, int perm) {
  // Round va down to a page address
  va = ROUNDDOWN(va, PGSIZE);
  // Use walk_pgtable to find the correct page table entry
  pte_t* pte = walk_pgdir(pgdir, (void*)va, 1);
  // Check if the address is already present
  if (*pte & PTE_P) {
    panic("remap");
  }
  // Add the pte for the mapping with permissions specified by perm
  *pte = pa | perm | PTE_P;
  // Flush the TLB to ensure the new mapping is visible
  lcr3(rcr3());
  return 0;
}


// Remove npages of mappings starting from va. va must be
// page-aligned. The mappings must exist.
// Optionally free the physical memory.
void
vm_page_remove(pagetable_t pagetable, uint64 va, uint64 npages, int do_free)
{
  // Verify that the virtual address is page-aligned
  assert(PAGE_ALIGNED(va));

  // Iterate through all pages to be removed
  for (size_t i = 0; i < npages; i++) {
    // Find the corresponding page table entry
    pte_t* pte = walk_pgdir(pgdir, va, 0);
    if (pte == NULL || !(*pte & PTE_P)) {
      // If the PTE is not found or the page is not present, panic
      panic("vm_page_remove: page not present\n");
    }

    // Clear the page table entry to remove the mapping
    *pte = 0;

    // Optionally free the physical page
    if (do_free) {
      physaddr_t pa = PTE_ADDR(*pte);
      page_free(pa);
    }

    // Advance to the next page
    va += PAGE_SIZE;
  }
}


// Map a block of virtual memory which is size bytes long and begins
// at virtual address va. This function allocates page frames as
// needed. Each allocated page recieves permissions specified by perm.
// On success, this function returns 0, on failure it returns -1.
int
vm_map_range(pagetable_t pagetable, uint64 va, uint64 size, int perm)
{
    // Loop through each virtual address pages. Note that va is not
    // necessarily page alligned, and so you must round it down.
    // We will allocate a new physical page frame for each page, and
    // then use vm_page_insert to add the page to the table.
    // YOUR CODE HERE
    return -1;
}


///////////////////////////////////////////////////////////////////////////////
// Static Helper Functions
///////////////////////////////////////////////////////////////////////////////

// Make a direct-map page table for the kernel.
static pagetable_t
make_kernel_pagetable(void)
{
  pagetable_t kpgtbl;

  kpgtbl = (pagetable_t) vm_page_alloc();
  memset(kpgtbl, 0, PGSIZE);

  // uart registers
  kernel_map_pages(kpgtbl, UART0, UART0, PGSIZE, PTE_R | PTE_W);

  // virtio mmio disk interface
  kernel_map_pages(kpgtbl, VIRTIO0, VIRTIO0, PGSIZE, PTE_R | PTE_W);

  // PLIC
  kernel_map_pages(kpgtbl, PLIC, PLIC, 0x400000, PTE_R | PTE_W);

  // map kernel text executable and read-only.
  kernel_map_pages(kpgtbl, KERNBASE, KERNBASE, (uint64)etext-KERNBASE, PTE_R | PTE_X);

  // map kernel data and the physical RAM we'll make use of.
  kernel_map_pages(kpgtbl, PGROUNDUP((uint64)etext), PGROUNDUP((uint64)etext),
  PHYSTOP-PGROUNDUP((uint64)etext), PTE_R | PTE_W);

  // map the trampoline for trap entry/exit to
  // the highest virtual address in the kernel.
  kernel_map_pages(kpgtbl, TRAMPOLINE, (uint64)trampoline, PGSIZE, PTE_R | PTE_X);


  return kpgtbl;
}



// Return the address of the PTE in page table pagetable
// that corresponds to virtual address va.  If alloc!=0,
// create any required page-table pages.
//
// The risc-v Sv39 scheme has three levels of page-table
// pages. A page-table page contains 512 64-bit PTEs.
// A 64-bit virtual address is split into five fields:
//   39..63 -- must be zero.
//   30..38 -- 9 bits of level-2 index.
//   21..29 -- 9 bits of level-1 index.
//   12..20 -- 9 bits of level-0 index.
//    0..11 -- 12 bits of byte offset within the page.
static pte_t * 
walk_pgtable(pagetable_t pagetable, uint64 va, int alloc)
{
  if(va >= MAXVA)
    panic("walk_pgtable");

  for(int level = 2; level > 0; level--) {
    pte_t *pte = &pagetable[PX(level, va)];
    if(*pte & PTE_V) {
      pagetable = (pagetable_t)PTE2PA(*pte);
    } else {
      if(!alloc || (pagetable = (pde_t*)vm_page_alloc()) == 0)
        return 0;
      memset(pagetable, 0, PGSIZE);
      *pte = PA2PTE(pagetable) | PTE_V;
    }
  }
  return &pagetable[PX(0, va)];
}


// add a mapping to the kernel page table.
// only used when booting.
// does not flush TLB or enable paging.
static void
kernel_map_pages(pagetable_t kpgtbl, uint64 va, uint64 pa, uint64 sz, int perm)
{
  if(kernel_map_range(kpgtbl, va, sz, pa, perm) != 0)
    panic("kernel_map_pages");
}


int kernel_map_range(pagetable_t pagetable, uintptr_t va, size_t size, physaddr_t pa) {
  size_t offset = va % PGSIZE;
  size_t remaining_size = size;
  uintptr_t cur_va = va;
  physaddr_t cur_pa = pa;
  while (remaining_size > 0) {
    // Get the page table entry for the current virtual address
    pte_t* pte = walk_pgdir(pagetable, (void*)cur_va, /*create*/1);
    if (pte == NULL) {
      return -1;
    }
    // Make sure the page table entry is not already present
    if (*pte & PTE_P) {
      panic("kernel_map_range: page table entry already present");
    }
    // Map the physical address to the virtual address
    *pte = cur_pa | PTE_P | PTE_W | PTE_U;
    // Advance to the next virtual and physical addresses
    size_t mapping_size = PGSIZE - offset;
    if (mapping_size > remaining_size) {
      mapping_size = remaining_size;
    }
    cur_va += mapping_size;
    cur_pa += mapping_size;
    offset = 0;
    remaining_size -= mapping_size;
  }
  return 0;
}


static void
free_range(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE) {
    vm_page_free(p);
  }
}


///////////////////////////////////////////////////////////////////////////////
// Unit Tests below this line. Do not change any part of the
// following!
///////////////////////////////////////////////////////////////////////////////

// Run unit tests on the virtual memory system
void
vm_test()
{
    void *p0, *p1, *p2;
    struct frame *fl;
    int passed;
    uint64 big_address = 0x90000000;
    int result;
    int perm = PTE_R | PTE_W;

    //allocation tests
    printf("vm_page_alloc test...");
    passed = 1;
    p0 = vm_page_alloc();
    p1 = vm_page_alloc();
    p2 = vm_page_alloc();
    if(!p0 || !p1 || !p2) {
        passed = 0;
    }

    //steal the free pages
    fl = frame_table;
    frame_table = 0;

    // try to allocate memory
    if(vm_page_alloc()) {
        passed = 0;
    }
    print_pass(passed);
    frame_table = fl;


    //deallocation test
    printf("vm_page_free test...");
    passed = 1;
    vm_page_free(p2);
    if(frame_table != p2) passed = 0;
    vm_page_free(p0);
    if(frame_table != p0) passed = 0;
    vm_page_free(p1);
    if(frame_table != p1) passed = 0;
    print_pass(passed);


    //lookup test
    printf("vm_lookup test...");
    passed = 1;
    if(vm_lookup(kernel_pagetable, (uint64)p0) != (uint64) p0) {
        passed = 0;
    }
    if(vm_lookup(kernel_pagetable, big_address) != 0) {
        passed = 0;
    }
    print_pass(passed);


    //insert test
    printf("vm_page_insert test...");
    passed = 1;
    p0 = vm_page_alloc();
    fl = frame_table;
    frame_table = 0;
    result = vm_page_insert(kernel_pagetable, big_address, (uint64)p0, perm);
    if(result != -1) {
        passed = 0;
    }
    frame_table = fl;
    result = vm_page_insert(kernel_pagetable, big_address, (uint64)p0, perm);
    if(result != 0 || vm_lookup(kernel_pagetable, big_address) == 0) {
    printf("fail 2\n");
        passed = 0;
    }
    *((int*)big_address) = 0x2a;
    if(*((int*)big_address) != 0x2a) {
    printf("fail 3\n");
        passed = 0;
    }
    print_pass(passed);


    //remove test
    printf("vm_page_remove test...");
    passed = 1;
    vm_page_remove(kernel_pagetable, big_address, 1, 1); 
    if(frame_table != p0){
        passed = 0;
    }
    if(vm_lookup(kernel_pagetable, big_address) != 0) {
        passed = 0;
    }
    p0 = vm_page_alloc();
    vm_page_insert(kernel_pagetable, big_address, (uint64)p0, perm);
    vm_page_remove(kernel_pagetable, big_address, 1, 0); 
    if(frame_table == p0) {
        passed = 0;
    }
    if(vm_lookup(kernel_pagetable, big_address) != 0) {
        passed = 0;
}
    print_pass(passed);
}
