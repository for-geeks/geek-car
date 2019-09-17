#include "share_memory.h"
 
int main()
{
	u8 buffer[] = {0x11, 0x12, 0x13, 0x14, 0x15};
	//u32 length = sizeof(buffer);
		
	CShareMemory csm("txh", 1024);
 
	while(1)
	{
		for(int i=0; i<5; i++)
		{
			buffer[i]++;
		}
		
		csm.PutToShareMem(buffer,5);
		usleep(100000);
	}
	return 0;
}

