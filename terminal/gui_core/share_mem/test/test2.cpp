#include "share_memory.h"
 
int main()
{
	//char buffer1[] = {0x11, 0x12, 0x13, 0x14, 0x15};
	unsigned char buffer[5] = {0};
	unsigned int length = sizeof(buffer);
		
	CShareMemory csm("txh", 1024);
 
	while(1)
	{
		csm.GetFromShareMem(buffer,length);
		for(int i=0; i<(int)length; i++)
		{
			printf("%x ", buffer[i]);
			/*if(buffer[i] != buffer1[i])
			{
				printf("T ---  buffer[%x]  buffer[%x]1\n", buffer[i] , buffer1[i]);
				return -1;
			}*/
		}
		printf("\n");
	}
	return 0;
}

