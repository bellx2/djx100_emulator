/* DJ-X100 emulator: run up to n instructions of the GNU RX core; the core leaves through longjmp on WAIT / STOP /
   exits, so the setjmp lives here in C. Returns RX_MAKE_STEPPED() or the stop code; *done = instructions run. */
#include "defs.h"
#include <setjmp.h>
#include "cpu.h"
extern jmp_buf decode_jmp_buf;
int decode_opcode (void);
/* optional PC hooks (flash 0xFFE00000..0xFFFFFFFF): called before the instruction executes */
unsigned char *rx_hook_map;
void (*rx_hook_fn) (unsigned int addr);
/* RAM hooks (code copied to RAM, e.g. the USB driver): a few exact PCs below 0x40000 */
unsigned int rx_ram_hooks[8];
int rx_n_ram_hooks;

int
rx_step_block (int n, int *done)
{
  volatile int k = 0;
  int rc = setjmp (decode_jmp_buf);
  if (rc != 0)
    {
      *done = k;
      return rc;
    }
  for (; k < n; k++)
    {
      int r;
      unsigned int hpc = (unsigned int) regs.r_pc;
      if (rx_hook_map && hpc >= 0xFFE00000u && rx_hook_map[hpc - 0xFFE00000u])
	rx_hook_fn (hpc);
      else if (hpc < 0x40000u && rx_n_ram_hooks)
	{
	  int h;
	  for (h = 0; h < rx_n_ram_hooks; h++)
	    if (rx_ram_hooks[h] == hpc)
	      {
		rx_hook_fn (hpc);
		break;
	      }
	  if ((unsigned int) regs.r_pc != hpc)
	    continue;		/* the hook redirected: next instruction */
	}
      r = decode_opcode ();
      if (!RX_STEPPED (r))
	{
	  *done = k;
	  return r;
	}
    }
  *done = k;
  return RX_MAKE_STEPPED ();
}
