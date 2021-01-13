/**
 * file:   platform.c
 * author: li xianjing <xianjimli@hotmail.com>
 * brief:  platform dependent function of stm32
 *
 * copyright (c) 2018 - 2018 Guangzhou ZHIYUAN Electronics Co.,Ltd. 
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * license file for more details.
 *
 */

/**
 * history:
 * ================================================================
 * 2018-05-12 li xianjing <xianjimli@hotmail.com> created
 *
 */

#include "tkc/mem.h"
#include "base/timer.h"
#include "tkc/platform.h"

#define MEM2_MAX_SIZE		6 * 1024 * 1024 
#define MEM2_ADDR           (uint8_t*)(0xC0000000 + 480 * 272 * 4)

ret_t platform_prepare(void) {
	timer_prepare(get_time_ms64);
	tk_mem_init(MEM2_ADDR, MEM2_MAX_SIZE);
	
	return RET_OK;
}


